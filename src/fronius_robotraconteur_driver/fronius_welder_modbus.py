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

def _uint16_to_sint16(u16_val):
    if  (0xF000 & u16_val) == 0:
        return u16_val
    else:
        return u16_val - 0x10000

def _uint16_to_float(u16_val, scale, signed = True):
    if signed:
        return float(_uint16_to_sint16(u16_val)) / float(scale)
    else:
        return float(u16_val) / float(scale)

def _bit(b):
    return 1 << b

def _unpack_flags(flags_in, reg_in, flag_map, flag_out_consts):
    for flag_name, flag_bit in flag_map:
        if (reg_in & _bit(flag_bit)) != 0:
            flags_in |= flag_out_consts[flag_name]
    return flags_in

class WelderImpl:
    def __init__(self, welder_ip, device_info, monitor_only = False):
        
        self.device_info = device_info

        self._welder_ip = welder_ip

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
        self._thread = None

    def RRServiceObjectInit(self, ctx, service_path):
        self._downsampler = RR.BroadcastDownsampler(ctx)
        self._downsampler.AddWireBroadcaster(self.device_clock_now)
        self._downsampler.AddWireBroadcaster(self.welder_state)

    def _start(self):
        with self._lock:
            self._keep_going = True
            self._thread = threading.Thread(target=self._run)
            self._thread.daemon = True
            self._thread.start()

    def _stop(self):
        assert self._keep_going
        with self._lock:
            self._keep_going = False
        self._thread.join()

    def _run(self):
        while self._keep_going:
            modbus = None
            try:
                modbus = ModbusTcpClient(self._welder_ip)
                modbus.connect()

                while self._keep_going:
                    read_res = modbus.read_holding_registers(0xF000,30)

                    read_regs = [read_res.getRegister(i) for i in range(30)]

                    state_struct = self._welder_modbus_regs_to_state(read_regs)

                    self.welder_state.OutValue = state_struct

                    time.sleep(0.1)
            except:
                traceback.print_exc()
                time.sleep(0.25)
            finally:
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
            "touch_collision_protection": 5,
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

        ret.state_flags = (hflags << 32) | flags

        ret.main_error = regs[8]
        ret.warning = regs[9]
        ret.welding_voltage = _uint16_to_float(regs[0xA],100)
        ret.welding_current = _uint16_to_float(regs(0xB),10)
        ret.motor_current_m1 = _uint16_to_float(regs[0xC],100)
        ret.motor_current_m2 = _uint16_to_float(regs[0xD],100)
        ret.motor_current_m3 = _uint16_to_float(regs[0xE],100)
        ret.wire_speed = _uint16_to_float(regs[0x10],100)
        ret.seam_tracking = _uint16_to_float(regs[0x11],10000, False)
        ret.welding_energy = _uint16_to_float(regs[0x12],10, False)
        ret.wire_position = _uint16_to_float(regs[0x13],100)

        return ret
    
def main():
    parser = argparse.ArgumentParser(description="Fronius Welder Power Source")

    parser.add_argument("--welder-ip", type=str, required=True, help="IP address of welder ModBus module (different than HTML interface)")
    parser.add_argument("--monitor-only", action="store_true", default=False, help="Only read the device feedback, do not send commands")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM orSIGINT (Linux only)")

    args, _ = parser.parse_known_args()

    RRC.RegisterStdRobDefServiceTypes(RRN)

    register_service_types_from_resources(RRN, __package__, ["experimental.fronius"])

    welder = WelderImpl(args.welder_ip, None, args.monitor_only)

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


