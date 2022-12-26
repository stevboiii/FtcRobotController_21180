/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are case sensitive.
 * Motors type: GoBILDA 312 RPM Yellow Jacket.
 *
 * Need config below hardware names:
 * 1. IMU - imu
 * 2. Distance sensor - fcds
 * 3. Distance sensor - rcds
 * 4. Wheel motors - names from initial function parameters.
 * 5. color sensor (Rev Color Sensor V3) - cs
 */
public class ChassisWith4Motors {
    //private
    HardwareMap hardwareMap = null;
    private final ElapsedTime runtime = new ElapsedTime();
    final double MAX_WAIT_TIME = 8.0; // in seconds
    private final boolean debugFlag = true;

    // Motors variables
    public DcMotor FrontLeftDrive = null;
    public DcMotor FrontRightDrive = null;
    public DcMotor BackLeftDrive = null;
    public DcMotor BackRightDrive = null;

    // Driving motor variables
    final double MAX_CORRECTION_POWER = 0.12;
    final double MAX_POWER = 1.0 - MAX_CORRECTION_POWER;
    final double RAMP_START_POWER = 0.35;
    final double RAMP_END_POWER = 0.2;
    final double SHORT_DISTANCE_POWER = 0.4;
    final double MIN_ROTATE_POWER = 0.21;
    final double AUTO_ROTATE_POWER = 0.9;

    // Position variables for autonomous
    final double COUNTS_PER_MOTOR_REV = 537.7;   // eg: GoBILDA 312 RPM Yellow Jacket
    final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    final double COUNTS_PER_INCH_DRIVE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); // Back-forth driving for 1 INCH. It is 42.8
    final double COUNTS_PER_INCH_STRAFE = 55; // robot strafe 1 INCH. the value is based on test

    // power control variables
    final double RAMP_UP_DISTANCE = 10.0; // ramp up in the first 10 inch
    final double RAMP_DOWN_DISTANCE = 10.0; // slow down in the final 9 inch
    final double SHORT_DISTANCE = 6.0; // consistent low power for short driving

    // imu
    private BNO055IMU imu = null;
    Orientation lastAngles = new Orientation();
    double globalAngle = 0.0;
    double correction = 0.0;
    double rotation = 0.0;
    PIDController pidRotate, pidDrive;
    boolean resetAngleFlag = false;
    double timeMS = 0.0;
    final int INERTIA_WAIT_TIME = 500; // in ms

    //sensors
    private DistanceSensor frontCenterDS = null;
    private DistanceSensor rightCenterDS = null;
    private ColorSensor colorSensor = null;


    /**
     * Init slider motors hardware, and set their behaviors.
     *
     * @param hardwareMap the Hardware Mappings.
     */
    public void init(HardwareMap hardwareMap,
                     String frontLeftMotorName, String frontRightMotorName,
                     String backLeftMotorName, String backRightMotorName) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;
        Logging.log("init driving motors.");

        FrontLeftDrive = hardwareMap.get(DcMotor.class, frontLeftMotorName);
        FrontRightDrive = hardwareMap.get(DcMotor.class, frontRightMotorName);
        BackLeftDrive = hardwareMap.get(DcMotor.class, backLeftMotorName);
        BackRightDrive = hardwareMap.get(DcMotor.class, backRightMotorName);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);

        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runWithoutEncoders(); // turn off encoder mode as default

        // IMU

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.014, .0, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.04, 0, 0);

        // make sure the imu gyro is calibrated before continuing.
        double loopStartTime = runtime.seconds();
        while (!imu.isGyroCalibrated() &&
                (runtime.seconds() - loopStartTime) < MAX_WAIT_TIME) {
            Thread.yield(); // idle
        }
        Logging.log("imu calib status", imu.getCalibrationStatus().toString());

        // Set up parameters for driving in a straight line.
        pidDrive.setInputRange(-90, 90);
        pidDrive.setSetpoint(0); // be sure input range has been set before
        pidDrive.setOutputRange(0, MAX_CORRECTION_POWER);
        pidDrive.enable();

        // Distance sensors
        frontCenterDS = hardwareMap.get(DistanceSensor.class, "fcds");
        rightCenterDS = hardwareMap.get(DistanceSensor.class, "rcds");
        colorSensor = hardwareMap.get(ColorSensor.class, "cs");

    }

    /**
     * Run distance with color sensor, distance sensor, and motor encoders
     * Run forward until see blue or red line
     * Turn robot
     * run forward until distance sensor reach cone.
     */
    public void runWithMultiSensors() {

        runUsingEncoders();
        float blue = (float)colorSensor.blue();
        float red = (float)colorSensor.red();
        float r1 = red;
        float b1 = blue;
        while ((red/r1 < 1.4) && (blue/b1 < 1.4) && (Math.abs(getEncoderDistance()) < 20))
        {
            drivingWithPID(-SHORT_DISTANCE_POWER, 0 ,0, false); // turn off PID to speed up while loop.
            r1 = red;
            b1 = blue;
            blue = (float)colorSensor.blue();
            red = (float)colorSensor.red();
            Logging.log("Red = %.1f, Blue = %.1f", red, blue);
        }
        setPowers(0);
        rotateIMUTargetAngle(90);
        runUsingEncoders();
        double dist = getFcDSValue();
        while ((dist > 4.5) && (Math.abs(getEncoderDistance()) < 20))
        {
            drivingWithPID(-SHORT_DISTANCE_POWER, 0 ,0, true);
            //Logging.log("Red = %d, Green = %d, Blue = %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            dist = getFcDSValue();
            Logging.log("Distance = %.2f", dist);
        }
        setPowers(0);
    }

    /**
     * Set to run to position mode for chassis motors
     *
     */
    private void runToPositionMode() {
        resetEncoders();
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Set chassis motors with run using encoders mode
     */
    private void runWithoutEncoders() {
        resetEncoders();
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Set chassis motors with run using encoders mode
     */
    private void runUsingEncoders() {
        resetEncoders();
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * resets encoder for motors
     */
    private void resetEncoders() {
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Set wheels motors target positions according to back-forward moving flag
     *
     * @param tPos: target position values for motors
     * @param isBF: flag for back-forward moving or left-right moving.
     *              Back forward(1), or left right (0)
     */
    private void setTargetPositions(int tPos, boolean isBF) {
        if (isBF) {
            FrontLeftDrive.setTargetPosition(tPos);
            FrontRightDrive.setTargetPosition(tPos);
            BackLeftDrive.setTargetPosition(tPos);
            BackRightDrive.setTargetPosition(tPos);
        } else {// move left or right, positive for right
            FrontLeftDrive.setTargetPosition(tPos);
            FrontRightDrive.setTargetPosition(-tPos);
            BackLeftDrive.setTargetPosition(-tPos);
            BackRightDrive.setTargetPosition(tPos);
        }
    }

    /**
     * Set wheels motors power
     *
     * @param p: the power value set to motors (0.0 ~ 1.0)
     */
    public void setPowers(double p) {
        p = Range.clip(p, -1, 1);
        FrontLeftDrive.setPower(p);
        FrontRightDrive.setPower(p);
        BackLeftDrive.setPower(p);
        BackRightDrive.setPower(p);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Check the first angle of IMU orientation in x-y plane
     * @return the value of first angle from IMU orientation
     */
    public double getIMUFirstAngle() {
        Orientation imuAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return imuAngles.firstAngle;
    }

    /**
     * Rotate left or right to make IMU direct to a certain degree.
     *
     * @param imuTargetAngle the target angle of imu after rotation.
     */
    public void rotateIMUTargetAngle(double imuTargetAngle) {
        double imuFirstAngle = getIMUFirstAngle();
        rotate(-AngleUnit.DEGREES.normalize(imuFirstAngle) + imuTargetAngle);
        if (debugFlag) {
            Logging.log("imu angle before rotation: %.2f", imuFirstAngle);
        }
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(double degrees) {
        resetAngle();
        runWithoutEncoders(); // make sure it is the mode of Run without encoder
        double power = AUTO_ROTATE_POWER;
        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359.99) {
            degrees = Math.floorMod(360, (int) degrees);
        }

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setInputRange(0, degrees);
        pidRotate.setSetpoint(degrees); // be sure input range has been set before
        pidRotate.setOutputRange(MIN_ROTATE_POWER, power);
        pidRotate.setTolerance(1.5);
        pidRotate.enable();

        if (Math.abs(degrees) < 1.0) {
            return;
        }

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.
        double maxLoopDuration = runtime.milliseconds();
        do {
            int[] motorsPos = {0, 0, 0, 0};
            double[] motorsPowerCorrection = {0.0, 0.0, 0.0, 0.0};
            double[] motorPowers = {power, power, power, power};
            motorsPos[0] = FrontLeftDrive.getCurrentPosition();
            motorsPos[1] = FrontRightDrive.getCurrentPosition();
            motorsPos[2] = BackLeftDrive.getCurrentPosition();
            motorsPos[3] = BackRightDrive.getCurrentPosition();

            calculateRotatePowerCorrection(motorsPos, motorsPowerCorrection);

            power = pidRotate.performPID(getAngle()); // power will be + on left turn.

            for (int i = 0; i < 4; i++) {
                motorPowers[i] = Range.clip(Math.abs(power) + motorsPowerCorrection[i],
                        0.0, Math.abs(power) * 1.1);
                motorPowers[i] = Math.copySign(Math.min(motorPowers[i], 1.0), power);
            }
            if (debugFlag) {
                Logging.log("Positions: FL = %d, FR = %d, BL = %d, BR = %d",
                        motorsPos[0], motorsPos[1], motorsPos[2], motorsPos[3]);
                Logging.log("Powers: FL = %.2f, FR = %.2f, BL = %.2f, BR = %.2f",
                        -motorPowers[0], motorPowers[1], -motorPowers[2], motorPowers[3]);
            }
            FrontLeftDrive.setPower(-motorPowers[0]);
            FrontRightDrive.setPower(motorPowers[1]);
            BackLeftDrive.setPower(-motorPowers[2]);
            BackRightDrive.setPower(motorPowers[3]);

        } while ((!pidRotate.onAbsTarget()) && ((runtime.milliseconds() - maxLoopDuration) < 1500));

        // turn the motors off.
        rightMotorSetPower(0);
        leftMotorSetPower(0);

        rotation = getAngle();
        if (debugFlag) {
            Logging.log("IMU angle before turn stop %.2f.", lastAngles.firstAngle);
            Logging.log("Rotated angle is %.2f.", rotation);
        }

        // reset angle tracking on new heading.
        resetAngle();
        Logging.log("Required turning degrees: %.2f.", degrees);
        Logging.log("IMU angle after turning stop is %.2f.", lastAngles.firstAngle);
    }

    /**
     * Set left side motors power.
     *
     * @param p the power set to front left motor and back left motor
     */
    private void leftMotorSetPower(double p) {
        p = Range.clip(p, -1, 1);
        FrontLeftDrive.setPower(p);
        BackLeftDrive.setPower(p);
    }

    /**
     * Set right side motors power.
     *
     * @param p the power set to front left right motor and back right motor
     */
    private void rightMotorSetPower(double p) {
        p = Range.clip(p, -1, 1);
        FrontRightDrive.setPower(p);
        BackRightDrive.setPower(p);
    }

    /**
     * Set front motors power.
     *
     * @param p the power set to front left right motor and front right motor
     */
    private void frontMotorSetPower(double p) {
        p = Range.clip(p, -1, 1);
        FrontRightDrive.setPower(p);
        FrontLeftDrive.setPower(p);
    }

    /**
     * Set front motors power.
     *
     * @param p the power set to back left right motor and back right motor
     */
    private void backMotorSetPower(double p) {
        p = Range.clip(p, -1, 1);
        BackRightDrive.setPower(p);
        BackLeftDrive.setPower(p);
    }

    /**
     * Set motors power and drive or strafe robot straightly with run_to_position mode by PID control.
     *
     * @param tDistance   the target distance in inch
     * @param targetSign: Input value for the target distance sign to indicate drive directions. Disable PID if it is zero.
     * @param isBF:       flag for back-forth (true) moving, or left-right moving (false)
     */
    private void setPowerWithPIDControl(double tDistance, int targetSign, boolean isBF) {
        double curTime = runtime.seconds();
        boolean speedRampOn = false;
        double drivePower = SHORT_DISTANCE_POWER;
        double tDistanceAbs = Math.abs(tDistance);

        if (tDistanceAbs > SHORT_DISTANCE) {
            speedRampOn = true;
        }
        correction = 0.0;
        setPowers(RAMP_START_POWER); // p is always positive for RUN_TO_POSITION mode.
        sleep(100); // let motors to start moving before checking isBusy.

        // in seconds
        while (robotIsBusy() || (getEncoderDistance(isBF) / tDistanceAbs < 0.5) &&
                ((runtime.seconds() - curTime) < MAX_WAIT_TIME)) {
            if (0 != targetSign) { // no pid if sign = 0;
                correction = pidDrive.performPID(getAngle());
            }

            if (speedRampOn) {
                double currDistance = Math.abs(getEncoderDistance(isBF));
                drivePower = MAX_POWER;
                double rampUpPower = MAX_POWER;
                double rampDownPower = MAX_POWER;
                //speed ramp up.
                if (currDistance < RAMP_UP_DISTANCE) {
                    rampUpPower = (MAX_POWER - RAMP_START_POWER) * currDistance / RAMP_UP_DISTANCE + RAMP_START_POWER;
                    rampUpPower = Range.clip(rampUpPower, RAMP_START_POWER, drivePower);
                }

                // speed ramp down
                if ((tDistanceAbs - currDistance) < RAMP_DOWN_DISTANCE) {
                    rampDownPower = (MAX_POWER / 2.5 - RAMP_END_POWER) * (tDistanceAbs - currDistance) / RAMP_DOWN_DISTANCE + RAMP_END_POWER;
                    rampDownPower = Range.clip(rampDownPower, RAMP_END_POWER, drivePower);
                }

                drivePower = Math.min(rampUpPower, rampDownPower);

                if (debugFlag) {
                    Logging.log("tDistance = %.2f, currLocation = %.2f", tDistance, currDistance);
                    Logging.log("rampUpPower = %.2f, rampDownPower = %.2f", rampUpPower, rampDownPower);
                }
            }

            if (debugFlag) {
                Logging.log("power = %.2f, correction = %.2f, global angle = %.3f, last angle = %.2f",
                        drivePower, correction, getAngle(), lastAngles.firstAngle);
            }

            if (isBF) { // left motors have same power
                leftMotorSetPower(drivePower - correction * targetSign);
                rightMotorSetPower(drivePower + correction * targetSign);
            } else { // front motors have same power
                frontMotorSetPower(drivePower - correction * targetSign);
                backMotorSetPower(drivePower + correction * targetSign);
            }
        }
        setPowers(0.0); //stop moving
        Logging.log("Drive power = %.2f, global angle = %.3f", drivePower, getAngle());
    }

    /**
     * Check if robot motors are busy. Return ture if yes, false otherwise.
     */
    private boolean robotIsBusy() {
        return (FrontRightDrive.isBusy() && FrontLeftDrive.isBusy() &&
                BackLeftDrive.isBusy() && BackRightDrive.isBusy());
    }

    /**
     * Calculate the motors power adjustment during rotation to make sure each motor has same
     * position counts, in order to avoid robot center shift during turning.
     *
     * @param pos                   the input of current positions for driving motors.
     * @param motorsPowerCorrection the output of power correction.
     */
    private void calculateRotatePowerCorrection(int[] pos, double[] motorsPowerCorrection) {
        int posAve = 0;
        for (int t = 0; t < 4; t++) {
            pos[t] = Math.abs(pos[t]);
            posAve += pos[t];
        }
        posAve = posAve / 4;

        // Do not correct power at the beginning when positions are small to avoid big impact on power.
        if (posAve < 50) {
            return;
        }

        for (int i = 0; i < 4; i++) {
            // the factor 4.0 below is according to test results. It can be adjust for sensitivity.
            motorsPowerCorrection[i] = (posAve - pos[i]) * 4.0 / posAve * AUTO_ROTATE_POWER;
        }
    }

    /**
     * Set target position for every wheel motor, and set power to motors to move the robot.
     * Turn off encode mode after moving. No action if moving distance less than 0.4inch (1cm).
     *
     * @param targetDistance: Input value for the target distance in inch.
     * @param isBackForth:    flag for back-forth (true) moving, or left-right moving (false)
     */
    public void runToPosition(double targetDistance, boolean isBackForth) {

        if (Math.abs(targetDistance) < 0.4) {
            return;
        }
        double countsPerInch = isBackForth? COUNTS_PER_INCH_DRIVE : COUNTS_PER_INCH_STRAFE;
        int targetPosition = (int) (targetDistance * countsPerInch);
        int tSign = (int) Math.copySign(1, targetDistance);
        setTargetPositions(targetPosition, isBackForth);

        runToPositionMode(); // turn on encoder mode, and reset encoders

        setPowerWithPIDControl(targetDistance, tSign, isBackForth);

        runWithoutEncoders(); // turn off encoder mode

        if (debugFlag) {
            Logging.log("Required moving distance %.2f.", targetDistance);
            Logging.log("Target Position = %d", targetPosition);
            Logging.log("PID = %d", tSign);
            Logging.log("Current Position after moving, FL = %d, FR= %d, BL = %d, BR = %d",
                    FrontLeftDrive.getCurrentPosition(), FrontRightDrive.getCurrentPosition(),
                    BackLeftDrive.getCurrentPosition(), BackRightDrive.getCurrentPosition());
        }
    }
    //used in autonomous to go to a certain junction, the distance the robot will have to run is larger than EncoderRange
    public void runToJunction(double EncoderRange, double DSrange, double Sensor) {
        runUsingEncoders();
        double driveDirection = Math.copySign(1, EncoderRange);

        EncoderRange = Math.abs(EncoderRange);
        double currEnDist = 0.0;

        // controlled by encoders
        while(EncoderRange - currEnDist > 0) {
            drivingWithPID(-SHORT_DISTANCE_POWER * driveDirection, 0.0, 0.0, true);
            currEnDist = Math.abs(getEncoderDistance());
        }

        // controlled by distance sensor
        Logging.log(" curEnrDis = %.2f, Curr Sensor dist = %.2f",
                Math.abs(getEncoderDistance()), getRcDsValue());
        while (getRcDsValue() > Sensor && getEncoderDistance() < EncoderRange + DSrange ) {
            drivingWithPID(-SHORT_DISTANCE_POWER / 2 * driveDirection, 0.0, 0.0, true);
            Logging.log("getRcDsValue = %.2f", getRcDsValue());
        }
        Logging.log("getRcDsValue = %.2f", getRcDsValue());
        setPowers(0.0);
        runWithoutEncoders();
    }

    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted.
     * This is simple shorthand for the operating-system-provided sleep() method.
     *
     * @param milliseconds amount of time to sleep, in milliseconds
     */
    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * drive power calculation and setting.
     *
     * @param drive power from drive button
     * @param turn power from turn button
     * @param strafe power from strafe button
     * @param PIDEnabled flag to enable/disable PID correction.
     */
    public void drivingWithPID(double drive, double turn, double strafe, boolean PIDEnabled) {
        double FrontLeftPower;
        double FrontRightPower;
        double BackLeftPower;
        double BackRightPower;

        // only enable correction when the turn button is not pressed.
        if (Math.abs(turn) > Math.ulp(0)) {
            pidDrive.reset();
            timeMS = runtime.milliseconds();
            resetAngleFlag = true;
            resetAngle(); // Resets the cumulative angle tracking to zero.
        }

        // turn on PID after a duration time to avoid robot inertia after turning.
        if (((runtime.milliseconds() - timeMS) > INERTIA_WAIT_TIME) && resetAngleFlag) {
            resetAngleFlag = false;
            resetAngle(); // Resets the cumulative angle tracking to zero.
            pidDrive.enable();
        }

        // Use PID with imu input to drive in a straight line.
        if (PIDEnabled && ((Math.abs(drive) > Math.ulp(0)) || (Math.abs(strafe) > Math.ulp(0)))) {
            correction = pidDrive.performPID(getAngle());
        } else {
            correction = 0.0;
        }

        FrontLeftPower = Range.clip(-drive - turn - strafe - correction, -1, 1);
        FrontRightPower = Range.clip(-drive + turn + strafe + correction, -1, 1);
        BackLeftPower = Range.clip(-drive - turn + strafe - correction, -1, 1);
        BackRightPower = Range.clip(-drive + turn - strafe + correction, -1, 1);

        // Send calculated power to wheels
        FrontLeftDrive.setPower(FrontLeftPower);
        FrontRightDrive.setPower(FrontRightPower);
        BackLeftDrive.setPower(BackLeftPower);
        BackRightDrive.setPower(BackRightPower);

        Logging.log("Motors power - FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                FrontLeftPower, FrontRightPower, BackLeftPower, BackRightPower);
    }

    /**
     * Get the average power of driving motors.
     * @return the average power of 4 driving motors.
     */
    public double getAveragePower() {
        return (Math.abs(FrontLeftDrive.getPower()) + Math.abs(FrontRightDrive.getPower()) +
                Math.abs(BackLeftDrive.getPower()) + Math.abs(BackRightDrive.getPower()))/4.0;
    }

    /**
     * Get the front center distance sensor value.
     * @return the value of front center distance, in inch
     */
    public double getFcDSValue() {
        return frontCenterDS.getDistance(DistanceUnit.INCH);
    }
    public double getRcDsValue() {
        return rightCenterDS.getDistance(DistanceUnit.INCH);
    }

    /**
     * Driving robot through distance sensor
     *
     * @param targetDS the target distance for disntance sensor.
     * @param targetEncoder the target distance for motors encoders.
     * @param range only check the target distance from distance sensor within the range
     *             of (targetDistance- range) to (targetDistance + range)
     * @param tolerance the tolerance for distance sensor to stop robot
     */
    public void runWithEncoderAndDistanceSensor(double targetDS, double targetEncoder, double range, double tolerance) {
        runUsingEncoders();
        double driveDirection = Math.copySign(1, targetEncoder);

        targetEncoder = Math.abs(targetEncoder);
        double currEnDist = 0.0;

        // controlled by encoders
        while(targetEncoder - currEnDist > range) {
            drivingWithPID(-SHORT_DISTANCE_POWER * driveDirection, 0.0, 0.0, true);
            currEnDist = Math.abs(getEncoderDistance());
            if (debugFlag) {
                Logging.log("drive Direction = %.2f", driveDirection);
                Logging.log("Target DS = %.2f, target Encoder = %.2f, currEnDis = %.2f, range = %.2f",
                        targetDS, targetEncoder, currEnDist, range);
                Logging.log(" curEnrDis = %.2f, Curr Sensor dist = %.2f",
                        Math.abs(getEncoderDistance()), getFcDSValue());
            }
        }

        // controlled by distance sensor
        double currDS = getFcDSValue();
        Logging.log(" curEnrDis = %.2f, Curr Sensor dist = %.2f",
                Math.abs(getEncoderDistance()), getFcDSValue());
        while (((currDS - targetDS) * driveDirection > tolerance) && (currEnDist - targetEncoder < range)) {
            drivingWithPID(-SHORT_DISTANCE_POWER * driveDirection, 0.0, 0.0, true);
            currEnDist = Math.abs(getEncoderDistance());
            currDS = getFcDSValue();
            if (debugFlag) {
                Logging.log("Target DS = %.2f, curEnrDis = %.2f, Curr Sensor dist = %.2f, tolerance = %.2f",
                        targetDS, currEnDist, currDS, tolerance);
            }
        }
        setPowers(0);
        runWithoutEncoders();
    }

    /**
     * check the current encoder positions of wheel motors
     * @return the average motors encoder position
     */
    private double getEncoderDistance() {
        return (FrontLeftDrive.getCurrentPosition() +
                FrontRightDrive.getCurrentPosition() +
                BackRightDrive.getCurrentPosition() +
                BackLeftDrive.getCurrentPosition()) / 4.0 / COUNTS_PER_INCH_DRIVE;
    }

    /**
     * check the current encoder positions of wheel motors
     * @return the average motors encoder position
     */
    private double getEncoderDistance( boolean isBF) {
        double dis;
        double aveEncoder = (Math.abs(FrontLeftDrive.getCurrentPosition()) +
                Math.abs(FrontRightDrive.getCurrentPosition()) +
                Math.abs(BackRightDrive.getCurrentPosition()) +
                Math.abs(BackLeftDrive.getCurrentPosition())) / 4.0;

        if (isBF) {
            dis = aveEncoder / COUNTS_PER_INCH_DRIVE;
        }
        else {
            dis = aveEncoder / COUNTS_PER_INCH_STRAFE;
        }
        return dis;
    }
}

