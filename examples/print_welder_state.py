# This example will read and print the state of a running welder driver

from RobotRaconteur.Client import *
import time

# Adjust the connection URL to the driver
sub = RRN.SubscribeService('rr+tcp://localhost:60823?service=welder')

consts = None

while True:

    res, c = sub.TryGetDefaultClientWait(5)
    if not res:
        print("Driver not available")
        continue

    if consts is None:    
        consts = RRN.GetConstants("experimental.fronius", c)
        flags_const = consts["WelderStateFlags"]
        hflags_const = consts["WelderStateHighFlags"]
    try:
        state, _ = c.welder_state.PeekInValue()
    except RR.ValueNotSetException:
        time.sleep(0.1)
        continue

    flags_str = []

    flags = state.welder_state_flags
    hflags = state.welder_state_flags >> 32

    for s, f in flags_const.items():
        if flags & f:
            flags_str.append(s)

    for s, f in hflags_const.items():
        if hflags & f:
            flags_str.append(s)

    print(f"flags: {', '.join(flags_str)}")

    print(f"welding_process: {state.welding_process}")
    print(f"main_error: {state.main_error}")
    print(f"warning: {state.warning}")
    print(f"welding_voltage: {state.welding_voltage}")
    print(f"welding_current: {state.welding_current}")
    print(f"wire_speed: {state.wire_speed}")
    print(f"welding_energy: {state.welding_energy}")
    print("")
    print("")

    time.sleep(0.5)