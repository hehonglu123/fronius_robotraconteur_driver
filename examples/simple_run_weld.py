# This example will run the welder for 2 seconds

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

