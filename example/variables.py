# Global Vars

import wpimath.geometry
import math

# SPEED CONFIG
kMaxSpeed = 3.5  # meters per second
kRMaxSpeed = 8.5 # 10
kTMaxSpeed = 3.0
kMaxAngularSpeed = math.pi  # 1/2 rotation per second
frontLeftZero = 0
frontRightZero = 0
backLeftZero = 0
backRightZero = 0
zeroThreshold = wpimath.geometry.Rotation2d(0.3)

# PID and FeedForward
drivePID_P = 0.02
drivePID_I = 0
drivePID_D = 0
driveFF_1 = 1.8
driveFF_2 = 3
turnPID_P = 7.8 #12 # 3
turnPID_I = 0
turnPID_D = 0.055 #0.1 #0.4750
turnFF_1 = 0 #0.05
turnFF_2 = 0 #0.45

TurnState = 0

# MODULE/CHASSIS CONFIG
chassisHalfLength = 0.32 # Chassis should be square

frontLeftDriveController = 4
frontLeftTurnController = 3
frontLeftDriveEncoder = 4
frontLeftTurnEncoder = 13

frontRightDriveController = 7
frontRightTurnController = 8
frontRightDriveEncoder = 7
frontRightTurnEncoder = 10

backRightDriveController = 5
backRightTurnController = 6
backRightDriveEncoder = 5
backRightTurnEncoder = 12

backLeftDriveController = 2
backLeftTurnController = 1
backLeftDriveEncoder = 2
backLeftTurnEncoder = 11

# CONTROLLERS/DEADBAND/SLEW
joystickPort1 = 2
joystickPort2 = 0

x_slewrate = 3
y_slewrate = 3
rot_slewrate = 1

x_deadband = 0.1
y_deadband = 0.1
rot_deadband = 0.2

triangleButton = 4
squareButton = 1
crossButton = 2
circleButton = 3
L1Button = 5
R1Button = 6
L2Button = 7
R2Button = 8
bigPad = 14
PSbutton = 13
shareButton = 9
optionsButton = 10

def setTurnState(rot) -> None:
    global TurnState
    if abs(rot) > 0:
        TurnState = 1
    else:
        TurnState = 0
    #print(TurnState)
    #return TurnState