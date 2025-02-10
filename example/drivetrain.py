#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import navx
import wpilib
import wpimath.geometry
import wpimath.kinematics
import swervemodule
import variables
from wpilib import Field2d, SmartDashboard
import ntcore
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds
from wpimath.estimator import SwerveDrive4PoseEstimator
import LimelightHelpers

'''
kMaxSpeed = 1.5  # meters per second
kRMaxSpeed = 0.1
kTMaxSpeed = 1.0
kMaxAngularSpeed = math.pi  # 1/2 rotation per second
frontLeftZero = 0
frontRightZero = 0
backLeftZero = 0
backRightZero = 0
zeroThreshold = wpimath.geometry.Rotation2d(0.3)
'''

class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:
        # NOTE: May need to tweak the center measurements 44.5mm rough measurement
        # NOTE: EVERYTHING IS MEASURE IN METERS! 
        # NOTE: Update center measure distance from each module
        # Wheel to Wheel = 63.4 cm
        # Chassis = 76cm
        '''
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.32, 0.32)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.32, -0.32)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.32, -0.32)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.32, 0.32)
        '''

        #self.field = Field2d()

        self.frontLeftLocation = wpimath.geometry.Translation2d(-variables.chassisHalfLength, variables.chassisHalfLength)
        self.frontRightLocation = wpimath.geometry.Translation2d(-variables.chassisHalfLength, -variables.chassisHalfLength)
        self.backRightLocation = wpimath.geometry.Translation2d(variables.chassisHalfLength, -variables.chassisHalfLength)
        self.backLeftLocation = wpimath.geometry.Translation2d(variables.chassisHalfLength, variables.chassisHalfLength)

        self.frontLeft = swervemodule.SwerveModule(variables.frontLeftDriveController, variables.frontLeftTurnController, variables.frontLeftDriveEncoder, variables.frontLeftTurnEncoder)
        self.frontRight = swervemodule.SwerveModule(variables.frontRightDriveController, variables.frontRightTurnController, variables.frontRightDriveEncoder, variables.frontRightTurnEncoder)
        self.backRight = swervemodule.SwerveModule(variables.backRightDriveController, variables.backRightTurnController, variables.backRightDriveEncoder, variables.backRightTurnEncoder)
        self.backLeft = swervemodule.SwerveModule(variables.backLeftDriveController, variables.backLeftTurnController, variables.backLeftDriveEncoder, variables.backLeftTurnEncoder)

        SmartDashboard.putData("TurnPID", self.frontLeft.turningPIDController)
        #SmartDashboard.putData("TurnFF", self.frontLeft.turnFeedforward)

        nt = ntcore.NetworkTableInstance.getDefault()
        topic = nt.getStructArrayTopic("/SwerveStates", SwerveModuleState)
        commandState = nt.getStructArrayTopic("/CommandStates", SwerveModuleState)
        self.pub = topic.publish()
        self.cmd = commandState.publish()
        '''
        BERT NOTES:
        
        driveMotorID: int,
        turningMotorID: int,
        driveEncoderID: int,
        turningEncoderID: int,

        Front Left (+,+) 
            - Drive Motor: 4
            - Rotation Motor: 3
            - Drive Encoder: 4
            - Rotation Encoder: 13 
        
        Front Right (+,-) 
            - Drive Motor: 7
            - Rotation Motor: 8
            - Drive Encoder: 7
            - Rotation Encoder: 10

        Rear Left (-,+) 
            - Drive Motor: 2
            - Rotation Motor: 1
            - Drive Encoder: 2
            - Rotation Encoder: 11

        Rear Right (-,-) 
            - Drive Motor: 5
            - Rotation Motor: 6
            - Drive Encoder: 5
            - Rotation Encoder: 12

        '''

        #self.gyro = wpilib.AnalogGyro(0)
        self.angler = navx.AHRS.create_spi()
        #print("gyroscope = ", self.angler)
        self.gyroinit = self.angler.getAngle()
        self.gyroradiansinit = wpimath.units.degreesToRadians(self.gyroinit)
        #self.resetPose = self.odometry.resetPose(self.odometry.getPose())
        #self.getRobotRelativeSpeeds = wpilib.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(self.xSpeed, self.ySpeed, self.#rot, wpimath.geometry.Rotation2d(self.gyroradians))
        #print(self.getRobotRelativeSpeeds)
        # print("gyro", self.gyro)

        #NOTE: Just defining the fixed kinematics of the bot
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backRightLocation,
            self.backLeftLocation,
        )

        #NOTE: getPosition - need to determine position value - velocity and angle -
        #NOTE: Need to understand expected units/values returned - is it meters & radians?
        self.poseEstimator = SwerveDrive4PoseEstimator(
            self.kinematics, self.angler.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backRight.getPosition(),
                self.backLeft.getPosition(),
            ), wpimath.geometry.Pose2d(),
            [0.05, 0.05, math.pi / 36], [0.5, 0.5, math.pi / 6]
        )

        self.angler.reset()

        #self.heading_controller.enableContinuousInput(-math.pi, math.pi)

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """

        #NOTE
        self.xSpeed = xSpeed
        self.ySpeed = ySpeed
        self.rot = rot

        #if fieldRelative:
        self.updateOdometry()

        self.gyro = self.angler.getAngle()
        self.gyroradians = wpimath.units.degreesToRadians(self.gyro)

        #print("gyro", self.gyroradians)

        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, wpimath.geometry.Rotation2d(self.gyroradians)
                )
                if fieldRelative
                else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, variables.kMaxSpeed
        )
        
        #NOTE: Should we desaturate for Turning speed motors? 
        
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backRight.setDesiredState(swerveModuleStates[2])
        self.backLeft.setDesiredState(swerveModuleStates[3])

        self.pub.set([self.frontLeft.getState(),self.frontRight.getState(),self.backRight.getState(),self.backLeft.getState()])

        self.cmd.set([
            swerveModuleStates[0],
            swerveModuleStates[1],
            swerveModuleStates[2],
            swerveModuleStates[3]
        ])

        # SmartDashboard.putData("frontLeft", self.frontLeft)
        # SmartDashboard.putData("bLeft", self.backLeft.getState())
        # SmartDashboard.putData("fRight", self.frontRight.getState())
        # SmartDashboard.putData("bRight", self.backRight.getState())
    
    def updateOdometry(self, useAprilTags:bool=True) -> None:
        """Updates the field relative position of the robot."""
        self.poseEstimator.update(
            self.angler.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backRight.getPosition(),
                self.backLeft.getPosition(),
            )
        )

        if useAprilTags and LimelightHelpers.searchForLimelights(debug=True):
            LimelightHelpers.setRobotOrientation(
                "limelight",
                self.getRotation().degrees(), self.angler.getRate(), # current position, rate of change
                self.angler.getPitch(), self.angler.getRawGyroY(), # current position, rate of change
                self.angler.getRoll(), self.angler.getRawGyroX() # current position, rate of change
            )
            estimate: LimelightHelpers.PoseEstimate = LimelightHelpers.PoseEstimate.getRobotPoseEstimateBlueMT2("limelight")

            if estimate.tagCount > 0:
                self.poseEstimator.setVisionMeasurementStdDevs([0.7, 0.7, 9999999])
                self.poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestamp)
    
    #NOTE
    def follow_trajectory(self, sample):
        # Get the current pose of the robot
        #pose = self.get_pose()
        self.pose = Field2d()

        # Generate the next speeds for the robot
        speeds = ChassisSpeeds(
            sample.vx + swervemodule.drivePIDController.calculate(self.pose.X(), sample.x),
            sample.vy + swervemodule.drivePIDController.calculate(self.pose.Y(), sample.y),
            sample.omega + swervemodule.turningPIDController.calculate(self.pose.rotation().radians(), sample.heading)
        )

        # Apply the generated speeds
        self.drive_field_relative(speeds)


    def alignment(self) -> None:
        self.frontLeft.setDesiredState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(0)))
        self.frontRight.setDesiredState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(0)))
        self.backRight.setDesiredState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(0)))
        self.backLeft.setDesiredState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(0)))

    def autoTest(self) -> None:
        self.frontLeft.setDesiredState(wpimath.kinematics.SwerveModuleState(1, wpimath.geometry.Rotation2d(1)))
        self.frontRight.setDesiredState(wpimath.kinematics.SwerveModuleState(1, wpimath.geometry.Rotation2d(1)))
        self.backRight.setDesiredState(wpimath.kinematics.SwerveModuleState(1, wpimath.geometry.Rotation2d(1)))
        self.backLeft.setDesiredState(wpimath.kinematics.SwerveModuleState(1, wpimath.geometry.Rotation2d(1)))

    def getPose(self) -> wpimath.geometry.Pose2d:
        return self.poseEstimator.getEstimatedPosition()

    def getPosition(self) -> wpimath.geometry.Translation2d:
        return self.getPose().translation()

    def getRotation(self) -> wpimath.geometry.Rotation2d:
        return self.getPose().rotation()