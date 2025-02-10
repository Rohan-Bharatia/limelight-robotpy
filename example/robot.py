#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import drivetrain
import variables
import elevator
import ntcore
#import limelight
#import limelightresults
import json
import time
#import limelight
#import vision
#import camera
#import auto
import ntcore
from wpilib import SmartDashboard, Field2d
from cscore import CameraServer
from wpilib import SmartDashboard
import choreo

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.Joystick(variables.joystickPort1)
        self.controller2 = wpilib.Joystick(variables.joystickPort2)
        self.swerve = drivetrain.Drivetrain()
        #self.odometry = 

        self.elevator = elevator.Elevator(16, 17, [100, 200, 300, 400])
        #self.limelight = limelight.PoseEstimate(pose, timestamp, latency, tagCount, tagSpan, avgTagDist, avgTagArea, fiducials)


        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        # Speed limiters

        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(variables.x_slewrate)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(variables.y_slewrate)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(variables.rot_slewrate)

        self.timer = wpilib.Timer()
        self.fieldDrive = 1
        #CameraServer.startAutomaticCapture()

        self.field = Field2d()
        SmartDashboard.putData("field", self.field)
        #SmartDashboard.putData("Swerve", self.swerve)
        
        # Loads from deploy/choreo/myTrajectory.traj
        # ValueError is thrown if the file does not exist or is invalid
        '''
        try:
            self.trajectory = choreo.load_swerve_trajectory("myTrajectory") # 
        except ValueError:
        # If the trajectory is not found, ChoreoLib already prints to DriverStation
            pass
        '''

    def robotPeriodic(self):
        self.swerve.updateOdometry(True)

    #FUTURE
    def autonomousInit(self):
        if self.trajectory:
            # Get the initial pose of the trajectory
            initial_pose = self.trajectory.get_initial_pose(self.is_red_alliance())

            if initial_pose:
                # Reset odometry to the start of the trajectory
                self.drive_subsystem.reset_odometry(initial_pose)

        # Reset and start the timer when the autonomous period begins
        self.timer.restart()

        #self.autoSelected = self.chooser.getSelected()
        #print("Auto selected:" + self.autoSelected)

    def autonomousPeriodic(self) -> None:
        if self.trajectory:
            # Sample the trajectory at the current time into the autonomous period
            sample = self.trajectory.sample_at(self.timer.get(), self.is_red_alliance())

            if sample:
                self.drive_subsystem.follow_trajectory(sample)
    
    def is_red_alliance(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    def teleopPeriodic(self) -> None:

        #self.swerve.updateOdometry(True)
        self.field.setRobotPose(self.swerve.getPose())

        #print('pose =', self.botpose)
        #print("hw", self.hw)

        '''
        result = ll.get_latest_results()
        parsed_result = limelightresults.parse_results(result)
        if parsed_result is not None:
            print("valid targets: ", parsed_result.validity, ", pipelineIndex: ", parsed_result.pipeline_id,", Targeting Latency: ", parsed_result.targeting_latency)
            #for tag in parsed_result.fiducialResults:
            #    print(tag.robot_pose_target_space, tag.fiducial_id)
        time.sleep(1)  # Set this to 0 for max fps
        '''

        # CHANGE TO FIELD DRIVE VS BOT RELETIVE
        if self.controller.getRawButton(variables.crossButton) == 1:
            self.fieldDrive = 2
        if self.controller.getRawButton(variables.circleButton) == 1:
            self.fieldDrive = 1

        if self.fieldDrive == 2:
            self.driveWithJoystick(True)
        else:
            self.driveWithJoystick(False)
    
    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        # NOTE: Check if we need inversion here
        #if fieldRelative:
        #    self.swerve.updateOdometry(True)

        xSpeed = (
            self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(1), variables.x_deadband)
            )
            * variables.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        # NOTE: Check if we need inversion here
        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(0), variables.y_deadband)
            )
            * variables.kTMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.

        '''
        rot = (
            (-self.rotLimiter.calculate(
                wpimath.applyDeadband((self.controller.getRawAxis(3) + 1) / 2, variables.rot_deadband)
            )
            * variables.kRMaxSpeed) +
            (self.rotLimiter.calculate(
                wpimath.applyDeadband((self.controller.getRawAxis(4) + 1) / 2, variables.rot_deadband)
            )
            * variables.kRMaxSpeed)
        )
        '''
        rot = (
            (self.rotLimiter.calculate(
                -wpimath.applyDeadband((self.controller.getRawAxis(3) + 1) / 2, variables.rot_deadband)
            ) +
                wpimath.applyDeadband((self.controller.getRawAxis(4) + 1) / 2, variables.rot_deadband)
            )
            * variables.kRMaxSpeed)

        '''
        rot = (
            (self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(2), variables.rot_deadband)
            ) / 2)
            * variables.kRMaxSpeed
        )
        '''
        variables.setTurnState(rot)

        #self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
    

        