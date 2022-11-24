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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are case sensitive.
 * Motors type: GoBILDA 312 RPM Yellow Jacket.
 */


public class ChassisWith4Motors {
    //private
    HardwareMap hardwareMap = null;
    private final ElapsedTime runtime = new ElapsedTime();
    static final double MAX_WAIT_TIME = 8.0; // in seconds
    private final boolean debugFlag = true;

    // Motors variables
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;

    // Driving motor variables
    static final double MAX_CORRECTION_POWER = 0.12;
    static final double MAX_POWER = 1.0 - MAX_CORRECTION_POWER;
    static final double RAMP_START_POWER = 0.35;
    static final double RAMP_END_POWER = 0.2;
    static final double SHORT_DISTANCE_POWER = 0.35;
    static final double MIN_ROTATE_POWER = 0.21;
    static final double AUTO_ROTATE_POWER = 0.9;

    // Position variables for autonomous
    static final double COUNTS_PER_MOTOR_REV = 537.7;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH_DRIVE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); // Back-forth driving for 1 INCH. It is 42.8
    static final double COUNTS_PER_INCH_STRAFE = 55; // robot strafe 1 INCH. the value is based on test

    // power control variables
    static final double RAMP_UP_DISTANCE = 10.0; // ramp up in the first 10 inch
    static final double RAMP_DOWN_DISTANCE = 9.0; // slow down in the final 9 inch
    static final double SHORT_DISTANCE = 6.0; // consistent low power for short driving

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

    // variables for reading current encoder positions just after a turn.
    int frontLeftPos;
    int frontRightPos;
    int backLeftPos;
    int backRightPos;

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

        //reset encode number to zero
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotRunWithPositionModeOn(false); // turn off encoder mode as default

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

    }

    /**
     * Set wheels motors to stop and reset encode to set the current encoder position to zero.
     * And then set to run to position mode if withPositionMode is on.
     * Otherwise, set to run without encode mode.
     *
     * @param withPositionMode: flag for wheels motors run with position mode on,
     *                          or off(run without encode)
     */
    private void robotRunWithPositionModeOn(boolean withPositionMode) {
        if (withPositionMode) {
            FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            // set back to WITHOUT ENCODER mode
            FrontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
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
        resetEncoders();
        robotRunWithPositionModeOn(false); // make sure it is the mode of Run without encoder
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
            Logging.log("Required turning degrees: %.2f.", degrees);
            Logging.log("IMU angle before turn stop %.2f.", lastAngles.firstAngle);
            Logging.log("Rotated angle is %.2f.", rotation);
        }

        // reset angle tracking on new heading.
        resetAngle();
        frontLeftPos = FrontLeftDrive.getCurrentPosition();
        frontRightPos = FrontRightDrive.getCurrentPosition();
        backLeftPos = BackLeftDrive.getCurrentPosition();
        backRightPos = BackRightDrive.getCurrentPosition();

        Logging.log("IMU angle after turning stop is %.2f.", lastAngles.firstAngle);
    }

    /**
     * Set left side motors power.
     *
     * @param p the power set to front left motor and back left motor
     */
    private void leftMotorSetPower(double p) {
        FrontLeftDrive.setPower(p);
        BackLeftDrive.setPower(p);
    }

    /**
     * Set right side motors power.
     *
     * @param p the power set to front left right motor and back right motor
     */
    private void rightMotorSetPower(double p) {
        FrontRightDrive.setPower(p);
        BackRightDrive.setPower(p);
    }

    /**
     * Set front motors power.
     *
     * @param p the power set to front left right motor and front right motor
     */
    private void frontMotorSetPower(double p) {
        FrontRightDrive.setPower(p);
        FrontLeftDrive.setPower(p);
    }

    /**
     * Set front motors power.
     *
     * @param p the power set to back left right motor and back right motor
     */
    private void backMotorSetPower(double p) {
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
    private void driveWithPIDControl(double tDistance, int targetSign, boolean isBF) {
        double curTime = runtime.seconds();
        boolean speedRampOn = false;
        double drivePower = SHORT_DISTANCE_POWER;
        double tDistanceAbs = Math.abs(tDistance);

        if (tDistanceAbs > SHORT_DISTANCE) {
            speedRampOn = true;
        }
        correction = 0.0;
        setPowers(RAMP_START_POWER); // p is always positive for RUN_TO_POSITION mode.
        sleep(10); // let motors to start moving before checking isBusy.

        // in seconds
        while (robotIsBusy() && ((runtime.seconds() - curTime) < MAX_WAIT_TIME)) {
            if (0 != targetSign) { // no pid if sign = 0;
                correction = pidDrive.performPID(getAngle());
            }

            if (speedRampOn) {
                double currDistance = Math.abs(FrontLeftDrive.getCurrentPosition()) / COUNTS_PER_INCH_DRIVE;
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
                    rampDownPower = (MAX_POWER / 3 - RAMP_END_POWER) * (tDistanceAbs - currDistance) / RAMP_DOWN_DISTANCE + RAMP_END_POWER;
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
        Logging.log("Autonomous -power = %.2f, global angle = %.3f", drivePower, getAngle());
    }

    /**
     * Check if robot motors are busy. Return ture if yes, false otherwise.
     */
    private boolean robotIsBusy() {
        return (FrontRightDrive.isBusy() && FrontLeftDrive.isBusy() &&
                BackLeftDrive.isBusy() && BackRightDrive.isBusy());
    }

    /**
     * Calculate the robot center shift due to a big angle turn.
     *
     * @param shift output array address. The array is for robot center shift in horizontal and
     *              portrait direction related to robot position after turning. shift[0] is for the
     *              shift in horizontal direction, shift[1] is for the shift in portrait direction.
     *              After turning, the robot need to strafe shift[0] inch, and drive shift[1] inch to
     *              move the robot center back. Strafing to left when shift[0] less than zero. Driving
     *              back when shift[1] less than zero.
     */
    public void locationShiftCalculation(double[] shift) {

        /*
         * TODO: add the shift calculation according to the algorithm.
         */
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
        double countsPerInch = isBackForth ? COUNTS_PER_INCH_DRIVE : COUNTS_PER_INCH_STRAFE;
        int targetPosition = (int) (targetDistance * countsPerInch);
        int tSign = (int) Math.copySign(1, targetDistance);
        setTargetPositions(targetPosition, isBackForth);

        robotRunWithPositionModeOn(true); // turn on encoder mode,and reset encoders

        driveWithPIDControl(targetDistance, tSign, isBackForth);

        robotRunWithPositionModeOn(false); // turn off encoder mode

        if (debugFlag) {
            Logging.log("Required moving distance %.2f.", targetDistance);
            Logging.log("Target Position = %d", targetPosition);
            Logging.log("PID = %d", tSign);
            Logging.log("Current Position after moving, FL = %d, FR= %d, BL = %d, BR = %d",
                    FrontLeftDrive.getCurrentPosition(), FrontRightDrive.getCurrentPosition(),
                    BackLeftDrive.getCurrentPosition(), BackRightDrive.getCurrentPosition());
        }
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

    public void drivingWithPID(double drive, double turn, double strafe) {
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
        if ((Math.abs(drive) > Math.ulp(0)) || (Math.abs(strafe) > Math.ulp(0))) {
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

        Logging.log("Motors power", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                FrontLeftPower, FrontRightPower, BackLeftPower, BackRightPower);
    }
}
