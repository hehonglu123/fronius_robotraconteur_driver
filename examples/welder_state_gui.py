from RobotRaconteur.Client import *
import tkinter as tk
import time
import numpy as np
import sys

url = 'rr+tcp://localhost:60823?service=welder'

if len(sys.argv) > 1:
    url = sys.argv[1]

sub = RRN.SubscribeService(url)
wire_sub = sub.SubscribeWire('welder_state')
wire_sub.InValueLifespan = 1

c = sub.GetDefaultClientWait(15)
consts = RRN.GetConstants("experimental.fronius", c)
flags_const = consts["WelderStateFlags"]
hflags_const = consts["WelderStateHighFlags"]

root = tk.Tk()
root.title = "Fronius Welder State"

label = tk.Label(root, fg = "black", justify=tk.LEFT)
label.pack()
label.config(text="test")

last_error = None

def update_label():

    res, welder_state, _ = wire_sub.TryGetInValue()
    if not res:
        label.config(text = "Connection Lost")
        label.after(250, update_label)
        return

    flags_text = "Welder State Flags:\n\n"

    for flag_name, flag_code in flags_const.items():
        if flag_code & welder_state.welder_state_flags != 0:
            flags_text += flag_name + "\n"

    hflags = welder_state.welder_state_flags >> 32
    for flag_name, flag_code in hflags_const.items():
        if flag_code & hflags != 0:
            flags_text += flag_name + "\n"

    outputs_text = "Outputs:\n\n"

    outputs_text += f"welding_process: {welder_state.welding_process}\n"
    outputs_text += f"main_error: {welder_state.main_error}\n"
    outputs_text += f"warning: {welder_state.warning}\n"
    outputs_text += f"welding_voltage: {welder_state.welding_voltage}\n"
    outputs_text += f"welding_current: {welder_state.welding_current}\n"
    outputs_text += f"wire_speed: {welder_state.wire_speed}\n"
    outputs_text += f"welding_energy: {welder_state.welding_energy}\n"
    
    label.config(text = flags_text + "\n\n" + outputs_text)

    label.after(250, update_label)

    

label.after(250,update_label)
root.mainloop()
