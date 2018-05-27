## Contributor: Mayank Mittal
# Script to fly the spawned drone in lawn mower patter

'''
~ #       #########
^ #       #       #
| #       #       #
| #       #       #
Y #       #       #
| #       #       #
| #       #       #
v #       #       #
~ #########       #
  <-X_step->
'''

from AirSimClient import *
import sys
import time
import math

########################@#### Control Variables ######@#######################
# Note: In UnrealEngine the coordinate system is inverted
X = 242        # final x coordinates
Y = -230        # length of path traversed along x
# The paramters H and X_step need to changed to collect more data
H = -15         # height of flight
X_step = -7.5

# defining variables for flight
YAW = 0         # yaw of the drone while flying (set to 0 assuming)
V = 1           # speed of drone while flying
##############################################################################

# color class to prettify the terminal outputs being printed
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

#print in green
def printg(message = ''):
    if message != '':
        print (bcolors.OKGREEN + message + bcolors.ENDC)

#connecting to the client
client = MultirotorClient('127.0.0.1')
client.confirmConnection()
client.enableApiControl(True)

#arming the drone
if (client.isApiControlEnabled()):
    if (client.armDisarm(True)):
        print (bcolors.OKBLUE + "drone is armed" + bcolors.ENDC)
else:
     print (bcolors.FAIL + "failed to arm the drone" + bcolors.ENDC)
     sys.exit(1);

if (client.getLandedState() == LandedState.Landed):
    if (not client.takeoff(30)):
        print (bcolors.FAIL + "failed to reach takeoff altitude after 30 seconds" + bcolors.ENDC)
        sys.exit(1);
    print (bcolors.OKBLUE + "drone should now be flying..." + bcolors.ENDC)
else:
    #fix for GPS loss errors
    print (bcolors.WARNING + "it appears the drone is already flying" + bcolors.ENDC)
    print (bcolors.WARNING + "kindly restart the simulator to ensure proper flying" + bcolors.ENDC)
    client.wait_key('press any key to continue flying')

#to pause the drone for a while to stabilize the flying altitude
client.hover();
time.sleep(5);
client.moveToPosition(0, 0, H, V, 60, DrivetrainType.MaxDegreeOfFreedom, YawMode(False, YAW), -1, 0)
print (bcolors.OKBLUE + "moved to altitude " + str(H) + "with yaw=0" + bcolors.ENDC)


x = 0                   # initial x coordinates
y = Y
delay_x = abs(X_step / V)    # delay for path along y
delay_y = abs(Y / V)         # delay for path along x

while x < X:
    printg("starting to move to (%3d, %3d)" % (x,y))
    client.moveToPosition(x, y, H, V, delay_y, DrivetrainType.MaxDegreeOfFreedom, YawMode(False, YAW), -1, 0)

    x = x + X_step;

    printg("starting to move to (%3d, %3d)" % (x,y))
    client.moveToPosition(x, y, H, V, delay_x, DrivetrainType.MaxDegreeOfFreedom, YawMode(False, YAW), -1, 0)

    printg("starting to move to (%3d, %3d)" % (x,y))
    client.moveToPosition(x, 0, H, V, delay_y, DrivetrainType.MaxDegreeOfFreedom, YawMode(False, YAW), -1, 0)

    x = x + X_step;

    printg("starting to move to (%3d, %3d)" % (x,y))
    client.moveToPosition(x, 0, H, V, delay_x, DrivetrainType.MaxDegreeOfFreedom, YawMode(False, YAW), -1, 0)

#to record the time of flight of the drone
client.land(10);
