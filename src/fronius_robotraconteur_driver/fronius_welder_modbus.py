import time
import traceback
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.RobDef import register_service_types_from_resources

from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
from RobotRaconteurCompanion.Util.SensorDataUtil import SensorDataUtil
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil

import argparse
import threading
from contextlib import suppress

from pymodbus.client import ModbusTcpClient

import numpy as np

def _uint16_to_sint16(u16_val):
    if  (0x8000 & u16_val) == 0:
        return u16_val
    else:
        return u16_val - 0x10000

def _uint16_to_float(u16_val, scale, signed = True):
    if signed:
        return float(_uint16_to_sint16(u16_val)) / float(scale)
    else:
        return float(u16_val) / float(scale)

def _sint16_to_uint16(s16_val):
    if s16_val >= 0:
        assert s16_val <= 0x7FFF, "Value out of range"
        return s16_val
    else:
        assert s16_val >= -0x8000, "Value out of range"
        return 0x10000 + s16_val

def _clamp_uint16(u16_val):
    if u16_val < 0:
        return 0
    if u16_val > 0xFFFF:
        return 0xFFFF
    return u16_val

def _float_to_uint16(float_val, scale, signed = True):
    if signed:
        return _sint16_to_uint16(int(float_val*scale))
    else:
        return _clamp_uint16(int(float_val*scale))

def _bit(b):
    return 1 << b

def _unpack_flags(flags_in, reg_in, flag_map, flag_out_consts):
    for flag_name, flag_bit in flag_map.items():
        if (reg_in & _bit(flag_bit)) != 0:
            flags_in |= flag_out_consts[flag_name]
    return flags_in

class WelderImpl:
    def __init__(self, welder_ip, device_info, monitor_only = False):
        
        self.device_info = device_info

        self._welder_ip = welder_ip
        self._monitor_only = monitor_only

        self._seqno = 0

        self._downsampler = None

        self._welder_const = RRN.GetConstants("experimental.fronius")
        self._welder_state_flags = self._welder_const["WelderStateFlags"]
        self._welder_state_high_flags = self._welder_const["WelderStateHighFlags"]
        self._welder_state_type = RRN.GetStructureType("experimental.fronius.FroniusWelderState")

        self._lock = threading.Lock()

        self._enabled = False

        self._connected = False

        self._keep_going = False
        self._modbus_thread = None
        self._state_thread = None

        self._robot_ready = False
        self._start_weld = False
        self._job_number = 0
        self._program_number = 0
        self._gas_on = False
        self._wire_inching = False
        self._wire_retract = False
        self._torch_blowout = False
        self._touch_sensing = False
        self._error_reset = False

        self._arc_length_correction = 0.0
        self._puls_dynamik_correction = 0.0
        self.seam_number = 0
        self._penetration_stabilizer = 0.0
        self._arc_length_stabilizer = 0.0

        self._date_time_utc_type = RRN.GetPodDType('com.robotraconteur.datetime.DateTimeUTC')
        self._isoch_info = RRN.GetStructureType('com.robotraconteur.device.isoch.IsochInfo')

        self._datetime_util = DateTimeUtil(RRN)
        self._sensor_data_util = SensorDataUtil(RRN)

        self._command_lock = threading.Lock()

        self._recv_regs = None
        self._seqno = 0

    def RRServiceObjectInit(self, ctx, service_path):
        self._downsampler = RR.BroadcastDownsampler(ctx)
        self._downsampler.AddWireBroadcaster(self.device_clock_now)
        self._downsampler.AddWireBroadcaster(self.welder_state)

    def _start(self):
        with self._lock:
            self._keep_going = True
            self._modbus_thread = threading.Thread(target=self._run_modbus)
            self._state_thread = threading.Thread(target=self._run_state_update)
            self._modbus_thread.daemon = True
            self._state_thread.daemon = True
            self._modbus_thread.start()
            self._state_thread.start()

    def _stop(self):
        assert self._keep_going

        self._robot_ready = False
        self._start_weld = False
        self._gas_on = False
        self._wire_inching = False
        self._wire_retract = False
        self._torch_blowout = False
        self._touch_sensing = False
        self._error_reset = False

        time.sleep(1)

        with self._lock:
            self._keep_going = False
        self._modbus_thread.join()

    def _run_state_update(self):
        rate = RRN.CreateRate(100)
        while self._keep_going:
            self._seqno += 1
            s = self._recv_regs
            ts = self._datetime_util.TimeSpec3Now()
            if s is None:
                state_struct = self._welder_state_type()
                state_struct.welder_state_flags |= self._welder_state_flags["communication_failure"]
                state_struct.seqno = self._seqno
                state_struct.ts = ts
                self.welder_state.OutValue = state_struct
            else:
                state_struct = self._welder_modbus_regs_to_state(s)
                state_struct.seqno = self._seqno
                state_struct.ts = ts
                self.welder_state.OutValue = state_struct

            rate.Sleep()

    def _run_modbus(self):
        rate = RRN.CreateRate(100)
        while self._keep_going:
            modbus = None
            self._recv_regs = None
            try:
                modbus = ModbusTcpClient(self._welder_ip)
                modbus.connect()

                while self._keep_going:
                    if self._monitor_only:
                        read_res = modbus.read_holding_registers(0xF100,30)
                        read_regs = [read_res.getRegister(i) for i in range(30)]
                    else:
                        control_reg = self._current_command_to_regs()
                        read_res = modbus.readwrite_registers(0xF100, 30, 0xF000, write_registers=control_reg[0:30])
                        if read_res.isError():
                            raise read_res
                        assert(read_res.function_code < 0x80)
                        read_regs = read_res.registers

                    # state_struct = self._welder_modbus_regs_to_state(read_regs)

                    # self.welder_state.OutValue = state_struct
                    self._recv_regs = read_regs

                    # time.sleep(0.1)
                    rate.Sleep()
            except:
                self._recv_regs = None
                traceback.print_exc()
                time.sleep(0.25)
            finally:
                self._recv_regs = None
                with suppress(Exception):
                    modbus.close()

    def _welder_modbus_regs_to_state(self, regs):
        ret = self._welder_state_type()

        flags = 0
        hflags = 0

        # Robot ready
        if self._enabled:
            flags |= self._welder_state_flags["enabled"]

        # Read 0xF101 Status Flag Group 2

        flags = _unpack_flags(flags, regs[1], {
            "ready": 1
        }, self._welder_state_flags)


        hflags = _unpack_flags(hflags, regs[1], {
            "arc_stable": 2,
            "current_flow": 3,
            "main_current_signal": 4,
            "torch_collision_protection": 5,
            "touched": 8,
            "torchbody_connected": 9,
            "command_out_of_range": 10,
            "correction_out_of_range": 11,
            "process_active": 12,
            "robot_motion_release": 13,
            "wire_stick_workpiece": 14
        }, self._welder_state_high_flags)

        hflags = _unpack_flags(hflags, regs[2], {
            "parameter_selection_internally": 8,
            "characteristic_number_valid": 9,
            "process_image_bit_0": 14,
            "process_image_bit_1": 15            
        }, self._welder_state_high_flags)

        hflags = _unpack_flags(hflags, regs[3], {
            "penetration_stabilizer": 0,
            "arclength_stabilizer": 1,          
        }, self._welder_state_high_flags)

        hflags = _unpack_flags(hflags, regs[4], {
            "notification": 14,
            "system_not_ready": 15,          
        }, self._welder_state_high_flags)

        hflags = _unpack_flags(hflags, regs[5], {
            "limit_signal": 0,
            "twin_sync_active": 9,
            "line_supply_status": 10,
            "warning": 14
        }, self._welder_state_high_flags)

        ret.welding_process = regs[2] & 0b11111
        ret.sensor_status = regs[4] & 0b111
        ret.safety_status = (regs[4] >> 11) & 0b11

        ret.welder_state_flags = (hflags << 32) | flags

        ret.main_error = regs[8]
        ret.warning = regs[9]
        ret.welding_voltage = _uint16_to_float(regs[0xA],100)
        ret.welding_current = _uint16_to_float(regs[0xB],10)
        ret.motor_current_m1 = _uint16_to_float(regs[0xC],100)
        ret.motor_current_m2 = _uint16_to_float(regs[0xD],100)
        ret.motor_current_m3 = _uint16_to_float(regs[0xE],100)
        ret.wire_speed = _uint16_to_float(regs[0x10],100)
        ret.seam_tracking = _uint16_to_float(regs[0x11],10000, False)
        ret.welding_energy = _uint16_to_float(regs[0x12],10, False)
        ret.wire_position = _uint16_to_float(regs[0x13],100)

        return ret

    def _current_command_to_regs(self):
        
        regs = [0]*32

        control_flags2 = 0
        if self._robot_ready:
            control_flags2 |= _bit(1)

        if self._start_weld:
            control_flags2 |= _bit(0)

        if self._gas_on:
            control_flags2 |= _bit(3)

        if self._error_reset:
            control_flags2 |= _bit(2)

        if self._wire_inching:
            control_flags2 |= _bit(4)

        if self._wire_retract:
            control_flags2 |= _bit(5)

        if self._torch_blowout:
            control_flags2 |= _bit(6)

        if self._touch_sensing:
            control_flags2 |= _bit(8)

        # TODO wire_sense
        regs[1] = control_flags2

        # working mode
        # TODO use parameter
        regs[8] = 2

        regs[9] = self._job_number
        regs[0xA] = self._program_number

        regs[0xB] = 0
        with suppress(RR.ValueNotSetException,AttributeError):
            regs[0xB] = _float_to_uint16(self.wire_feeder_command.InValue[0],100)

        with suppress(RR.ValueNotSetException,AttributeError):
            regs[0xF] = _float_to_uint16(self.welding_speed.InValue[0],10,False)

        return regs
    
    def prepare_welder(self):
        with self._command_lock:
            self._robot_ready = True
            time.sleep(0.1)

    def release_welder(self):
        self._robot_ready = False
        self._start_weld = False

    def start_weld(self):
        with self._command_lock:
            self._start_weld = True
    
    def stop_weld(self):
        self._start_weld = False

    def start_touch_sensing(self):
        with self._command_lock:
            self._touch_sensing = True

    def stop_touch_sensing(self):
        self._touch_sensing = False

    @property
    def job_number(self):
        return self._job_number
    @job_number.setter
    def job_number(self, value):
        assert value >= 0 and value <= 1000, "Job number out of range"
        with self._command_lock:
            self._job_number = value
            time.sleep(0.1)

    @property
    def program_number(self):
        return self._program_number
    @program_number.setter
    def program_number(self, value):
        assert value >= 0 and value <= 255, "Program number out of range"
        with self._command_lock:
            self._program_number = value
            time.sleep(0.1)

    def gas_on(self, duration_seconds):
        assert duration_seconds < 5
        with self._command_lock:
            self._gas_on = True
            time.sleep(duration_seconds)
            self._gas_on = False

    def gas_on(self, duration_seconds):
        assert duration_seconds < 5
        with self._command_lock:
            self._gas_on = True
            time.sleep(duration_seconds)
            self._gas_on = False

    def wire_inching(self, duration_seconds):
        assert duration_seconds < 5
        with self._command_lock:
            self._wire_inching = True
            time.sleep(duration_seconds)
            self._wire_inching = False
    
    def wire_retract(self, duration_seconds):
        assert duration_seconds < 5
        with self._command_lock:
            self._wire_retract = True
            time.sleep(duration_seconds)
            self._wire_retract = False

    def wire_inching(self, duration_seconds):
        assert duration_seconds < 5
        with self._command_lock:
            self._wire_inching = True
            time.sleep(duration_seconds)
            self._wire_inching = False

    @property
    def arc_length_correction(self):
        return self._arc_length_correction
    @arc_length_correction.setter
    def arc_length_correction(self,value):
        assert value >= -10 and value <= 10
        self._arc_length_correction = value

    @property
    def puls_dynamik_correction(self):
        return self._puls_dynamik_correction
    @puls_dynamik_correction.setter
    def puls_dynamik_correction(self, value):
        assert value >= -10 and value <= 10
        self._puls_dynamik_correction = value

    @property
    def penetration_stabilizer(self):
        return self._penetration_stabilizer
    @penetration_stabilizer.setter
    def penetration_stabilizer(self, value):
        assert value >= 0 and value <= 10
        self._penetration_stabilizer = value

    @property
    def arc_length_stabilizer(self):
        return self._arc_length_stabilizer
    @arc_length_stabilizer.setter
    def arc_length_stabilizer(self, value):
        assert value >= 0 and value <= 10
        self._arc_length_stabilizer = value

    def reset_errors(self):
        with self._command_lock:
            self._error_reset = True
            time.sleep(0.1)
            self._error_reset = False

    @property
    def isoch_downsample(self):
        return self._downsampler.GetClientDownsample(RR.ServerEndpoint.GetCurrentEndpoint())

    @isoch_downsample.setter
    def isoch_downsample(self, value):
        return self._downsampler.SetClientDownsample(RR.ServerEndpoint.GetCurrentEndpoint(),value)

    @property
    def isoch_info(self):
        ret = self._isoch_info()
        ret.update_rate = 100
        ret.max_downsample = 100
        ret.isoch_epoch = np.zeros((1,),dtype=self._date_time_utc_type)
    

def main():
    parser = argparse.ArgumentParser(description="Fronius Welder Power Source")

    parser.add_argument("--welder-ip", type=str, required=True, help="IP address of welder ModBus module (different than HTML interface)")
    parser.add_argument("--monitor-only", action="store_true", default=False, help="Only read the device feedback, do not send commands")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM orSIGINT (Linux only)")

    args, _ = parser.parse_known_args()

    RRC.RegisterStdRobDefServiceTypes(RRN)

    register_service_types_from_resources(RRN, __package__, ["experimental.fronius"])

    welder = WelderImpl(args.welder_ip, None, args.monitor_only)

    try:
        with RR.ServerNodeSetup("fronius.welder",60823):

            service_ctx = RRN.RegisterService("welder","experimental.fronius.FroniusWelder",welder)
            welder._start()

            if args.wait_signal:  
                #Wait for shutdown signal if running in service mode          
                print("Press Ctrl-C to quit...")
                import signal
                signal.sigwait([signal.SIGTERM,signal.SIGINT])
            else:
                #Wait for the user to shutdown the service
                input("Server started, press enter to quit...")
    finally:
        welder._stop()


