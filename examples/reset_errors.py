# This example will reset the errors of the welder 
from RobotRaconteur.Client import *
import time

# Adjust the connection URL to the driver
c = RRN.ConnectService('rr+tcp://localhost:60823?service=welder')

c.reset_errors()

