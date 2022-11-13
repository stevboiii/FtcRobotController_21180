Welcome!
This repository contains the public FTC SDK for the power play(2022-23) competition season by team 21180.

## Robot requirements

For this SDK to function, your robot needs the following components: 
- 4 DC motors for Mecanum wheels configured as "FrontLeftDrive", "FrontRightDrive", "BackLeftDrive", and "BackRightDrive"
- 1 servo for a passive intake configured as "clawServo"
- 2 4-stage viper slides with respective DC motors configured as "LeftSliderMotor" and "RightSliderMotor"
- A control hub with an IMU connected to an expansion hub
- You can configure the DC motors and the Servo through the "configure robot" option on your driver hub.
- To create a new configuration, see [This Link](https://www.youtube.com/watch?v=ME0G2-B72GE) for instructions.
- The robot will also require a V-shaped jig at the front of the robot 

## Autonomous

For the current state, use "AutonomousRight" as your autonomous configuration and "TeleopDualDrivers" as your tele-op.
The current version of autonomous requires 1 preloaded cone and places 3 cones onto the high junction closest to your side of the field.
To start off with autonomous, the robot needs to be placed precisely on the center line of the parking location against the wall. 
If placed correctly, a driver hub can precisely fit on either side of the robot within the mat.
AutonomousRight is tuned to using a custom signal sleeve with the color red as parking space 1, green as 2, and blue as 3.

## Controller requirements

This version of TeleopDualDrivers requires 2 identical controllers with at least:
- 2 sticks on either side
- A D-pad to the left
- A, B, X, and Y buttons on the right
- A bumper and a trigger on both sides
- A back button and a mode button towards the center of the controller

## Controls

The default for TeleopDualDrivers is the "dual driver mode" where control is split between two controllers.
Controller 1 controls the following:
- Driving back & forth with the left stick forward & back.
- Strafing with left stick left & right
- Automatic load* with the left bumper
- Toggle between single and dual driver modes with Back + Left Trigger
Controller 2 controls the following:
- Extending and retracting the arm with the left stick up & down
- Extending the arm to heights for loading, low junction, medium junction, and high junction with X, A, B, and Y respectively
- Opening and closing the claw with D-pad up and down
- Automatic unload** with the right bumper
You can move the arm when the robot is driving

* For this function to work, the cone needs to be aligned with the V jig
** For this function to work, the junction needs to be aligned with the V jig

## Resources

- [Our team website](https://stevenzh530.wixsite.com/ftcteam-error418)
- [FIRST tech challenge](https://www.firstinspires.org/robotics/ftc)
- [Power Play intro video](https://www.youtube.com/watch?v=HsitvZ0JaDc)
- [Playing field info and assembly](https://firstinspiresst01.blob.core.windows.net/first-energize-ftc/field-setup-and-assembly-guide.pdf)
- [Game manual part 1](https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/game-manual-part-1-traditional-events.pdf)
- [Game manual part 2](https://firstinspiresst01.blob.core.windows.net/first-energize-ftc/game-manual-part-2-traditional.pdf)