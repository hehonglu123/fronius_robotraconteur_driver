# Fronius Welder Robot Raconteur Driver

This project provides a Robot Raconteur driver for the "Fronius TPS 500i PULSE" weld power source 
(Part number 4,075,180,830) using the "RI FB/i Yaskawa WeldCom 2.0" communication module (Part number 42,0426,0218).
This driver communicates directly with the welder using ModBus TCP/IP. The driver can either be used to monitor
the welder and provide state data, or directly control the welder. When the driver is used to directly control
the welder, any configured communication from the robot controller should be disabled.

The following Fronius user manuals should be referenced for using this driver:

* [Operating Instruction TPS 320i, TPS 400i, TPS 500i, TPS 600i, TPS 400i LSC ADV PL](https://www.fronius.com/~/downloads/Perfect%20Welding/Operating%20Instructions/42,0426,0114,PL.pdf)
* [TPS/i Interface Signal Descriptions](https://manuals.fronius.com/html/4204260227/en-US.html)
* [RI FB/i Yaskawa WeldCom 2.0](https://www.fronius.com/~/downloads/Perfect%20Welding/Operating%20Instructions/42%2C0426%2C0218%2CDE.pdf)

**WARNING!! ROBOTIC WELDING CAN BE EXTREMELY HAZARDOUS!! CAREFULLY REVIEW SAFETY INFORMATION IN THE USER MANUALS!!**

**The instructions for this driver are not a substitute for carefully reading the user manuals!**

## Installation

The driver can be installed from GitHub:

```
python -m pip install --user git+https://github.com/johnwason/fronius_robotraconteur_driver
```

## Running the Driver

The driver can be run from the command line:

```
python -m fronius_robotraconteur_driver --welder-ip=192.168.1.51
```

Replace 192.168.1.51 with the IP address *of the ModBus TCP/IP communication module*. See the documentation
for the communication module for instructions on configuring the IP address. If WeldCom is configured on the
Yaskawa controller, the IP address can be read from the settings.

This driver can be used to monitor the welder state while WeldCom is enabled. Use the startup command line option
`--monitor-only` to disable commanding the welder. Without this flag, the driver will attempt to take command of
the welder.

By default the driver can be connected using the following url: `rr+tcp://localhost:60823?service=welder`.

The standard Robot Raconteur command line configuration flags are supported. See 
https://github.com/robotraconteur/robotraconteur/wiki/Command-Line-Options

## Using the Driver

**WARNING: ROBOTIC WELDING CAN BE EXTREMELY HAZARDOUS!! CAREFULLY REVIEW SAFETY INFORMATION IN THE USER MANUALS!!**

**The instructions for this driver are not a substitute for carefully reading the user manuals!**

The driver makes starting and stopping the weld quite simple:

```python
from RobotRaconteur.Client import *
import time

# Adjust the connection URL to the driver
c = RRN.ConnectService('rr+tcp://localhost:60823?service=welder')

# Set the job number to use for this weld
c.job_number = 13

# Prepare the welder. This takes a few hundred milliseconds
c.prepare_welder()
# Start the weld
c.start_weld()

# Wait for 2 seconds. The robot would move during this time
time.sleep(2)

# Stop the weld
c.stop_weld()

# Release the welder
c.release_welder()
```

See `examples/simple_run_weld.py`.

The welder can also be used to monitor the welder state. See `examples/print_welder_state.py` for an example
that prints the current welder state. See [TPS/i Interface Signal Descriptions](https://manuals.fronius.com/html/4204260227/en-US.html)
for details on what the state data means. The `examples/welder_state_gui.py` example provides a simple GUI displaying
real-time welder state.

If the welder is in an error state, the `reset_errors()` method may be used to reset the welder. See
`examples/reset_errors.py` for an example.

Various parameters can be configured. In general, the `job_number` is used, however there are other
configuration options available. See [RI FB/i Yaskawa WeldCom 2.0](https://www.fronius.com/~/downloads/Perfect%20Welding/Operating%20Instructions/42%2C0426%2C0218%2CDE.pdf)
to better understand the configuration options. Note that the driver applies scaling to the parameters,
so do not apply scaling when setting parameters.

## License

Apache 2.0



