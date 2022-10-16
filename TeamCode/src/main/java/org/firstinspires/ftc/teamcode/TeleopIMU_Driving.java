/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleopIMU_Driving", group="Concept")
//@Disabled
public class TeleopIMU_Driving extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;


    // Driving motor variables
    static final int RAMP_ON = 0; // ramp on: 1; off: 0
    static final double POWER_FACTOR = 0.6;  // used to adjust driving sensitivity.
    static final double ADJUST_POSITION_POWER = 0.2; // used for auto driving

    // slider motor variables
    private DcMotor SliderMotor = null;
    static final double SLIDER_MOTOR_POWER = 0.4; // slider string gets loose with too high speed
    static final int COUNTS_PER_INCH = 115;

    static final int FOUR_STAGE_SLIDER_MAX_POS = 4200;  // Leave 100 counts for buffer.
    static final int SLIDER_MIN_POS = 0;

    static final int GROUND_JUNCTION_POS = 400;
    static final int READY_FOR_GRIP_POSITION = 700; // lift a little bit to get ready for unloading

    // 10inch for low junction, 20inch for medium, and 30 for high
    static final int LOW_JUNCTION_POS = COUNTS_PER_INCH * 10 + READY_FOR_GRIP_POSITION; // need double check by testing
    static final int MEDIUM_JUNCTION_POS = COUNTS_PER_INCH * 20 + READY_FOR_GRIP_POSITION;
    static final int HIGH_JUNCTION_POS = COUNTS_PER_INCH * 30 + READY_FOR_GRIP_POSITION;

    static final int SLIDER_MOVE_DOWN_POSITION = 700; // move down a little bit to unload cone

    static final int POSITION_COUNTS_FOR_ONE_REVOLUTION = 512; // Approximate value from testing

    int motorPositionInc = POSITION_COUNTS_FOR_ONE_REVOLUTION/40; // set value based on testing
    int sliderMotorTargetPosition = 0;


    // claw servo motor variables
    private Servo clawServo = null;
    static final double CLAW_INCREMENT = 0.004;     // amount to slew servo each CYCLE_MS cycle
    static final double CLAW_OPEN_POS = 0.08;     // Maximum rotational position
    static final double CLAW_CLOSE_POS = 0.25;
    static final double CLAW_MAX_POS = CLAW_CLOSE_POS;
    static final double CLAW_MIN_POS = CLAW_OPEN_POS;     // Minimum rotational position
    double clawServoPosition = CLAW_OPEN_POS;


    // arm servo variables
    private Servo armServo = null;
    static final double ARM_INCREMENT = 0.0015;     // amount to slew servo each CYCLE_MS cycle
    static final double ARM_MAX_POS = 0.6;     // Maximum rotational position
    static final double ARM_MIN_POS = 0.1;     // Minimum rotational position
    static final double ARM_LOAD_POSITION = 0.3;
    static final double ARM_UNLOAD_POSITION = 0.3;
    double armServoPosition = ARM_LOAD_POSITION;


    // variables for auto load and unload cone
    static final int COUNTS_PER_FEET_MOTION = 360; // robot moving 1 feet for 360 counts position.
    double robotAutoLoadMovingDistance = 0.0; // in feet
    double robotAutoUnloadMovingDistance = 0.25; // in feet
    

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FrontLeftDrive  = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeftDrive = hardwareMap.get(DcMotor.class,"BackLeft");
        BackRightDrive = hardwareMap.get(DcMotor.class,"BackRight");
        SliderMotor = hardwareMap.get(DcMotor.class,"SliderMotor");
        armServo = hardwareMap.get(Servo.class, "ArmServo");
        clawServo = hardwareMap.get(Servo.class, "TestServo");

        // claw servo motor initial
        clawServoPosition = CLAW_INCREMENT;
        clawServo.setPosition(clawServoPosition);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);

        /* slider motor control */
        // based on how Motor installed on robot.
        SliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        SliderMotor.setTargetPosition(sliderMotorTargetPosition);
        // Reset slider motor encoder counts kept by the motor
        SliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Using encoder mode to run slider motor
        SliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set motor to run to target encoder position and top with brakes on.
        SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double FrontLeftPower;
            double FrontRightPower;
            double BackLeftPower;
            double BackRightPower;

            double drive = POWER_FACTOR * Math.pow(gamepad1.left_stick_y, 1 + (2 * RAMP_ON));
            double turn  =  POWER_FACTOR * Math.pow(-gamepad1.right_stick_x, 1 + (2 * RAMP_ON));
            double strafe = POWER_FACTOR * Math.pow(-gamepad1.left_stick_x, 1 + (2 * RAMP_ON));

            FrontLeftPower    = Range.clip(-drive - turn - strafe, -1, 1) ;
            FrontRightPower   = Range.clip(-drive + turn + strafe, -1, 1) ;
            BackLeftPower    = Range.clip(-drive - turn + strafe, -1, 1) ;
            BackRightPower   = Range.clip(-drive + turn - strafe, -1, 1) ;

            // Send calculated power to wheels
            FrontLeftDrive.setPower(FrontLeftPower);
            FrontRightDrive.setPower(FrontRightPower);
            BackLeftDrive.setPower(BackLeftPower);
            BackRightDrive.setPower(BackRightPower);

            // use Y button to lift up the slider reaching high junction
            if (gamepad1.y) {
                sliderMotorTargetPosition = HIGH_JUNCTION_POS;
            }

            // use B button to lift up the slider reaching medium junction
            if (gamepad1.b) {
                sliderMotorTargetPosition = MEDIUM_JUNCTION_POS;
            }

            // use A button to lift up the slider reaching low junction
            if (gamepad1.a) {
                sliderMotorTargetPosition = LOW_JUNCTION_POS;
            }

            // use X button to move the slider for ground junction position
            if (gamepad1.x) {
                sliderMotorTargetPosition = GROUND_JUNCTION_POS;
            }

            // use right stick_Y to lift or down slider continuously
            sliderMotorTargetPosition -= (int)((gamepad1.right_stick_y) * motorPositionInc);
            sliderMotorTargetPosition = Range.clip(sliderMotorTargetPosition, SLIDER_MIN_POS,
                    FOUR_STAGE_SLIDER_MAX_POS);
            telemetry.addData("Status", "slider motor Target position %d",
                    sliderMotorTargetPosition);

            SliderMotor.setTargetPosition(sliderMotorTargetPosition);
            SliderMotor.setPower(SLIDER_MOTOR_POWER); // slider motor start movement
            telemetry.addData("Status", "slider motor current position %d",
                    SliderMotor.getCurrentPosition());

            // Keep stepping up until we hit the max value.
            if (gamepad1.dpad_up) {
                clawServoPosition += CLAW_INCREMENT;
            }
            else if (gamepad1.dpad_down) {
                clawServoPosition -= CLAW_INCREMENT;
            }
            clawServoPosition = Range.clip(clawServoPosition, CLAW_MIN_POS, CLAW_MAX_POS);
            clawServo.setPosition(clawServoPosition);
            telemetry.addData("Status", "Claw Servo position %.2f", clawServoPosition);

            // arm servo motor control. Keep stepping up until we hit the max value.
            if (gamepad1.dpad_left) {
                armServoPosition += ARM_INCREMENT;
            }
            else if (gamepad1.dpad_right) {
                armServoPosition -= ARM_INCREMENT;
            }
            armServoPosition = Range.clip(armServoPosition, ARM_MIN_POS, ARM_MAX_POS);
            armServo.setPosition(armServoPosition);
            telemetry.addData("Status", "Arm Servo position %.2f", armServoPosition);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Frontleft (%.2f), Frontright (%.2f)," +
                    " Backleft (%.2f), Backright (%.2f)", FrontLeftPower, FrontRightPower,
                    BackLeftPower,BackRightPower);
            telemetry.update();

            //  auto driving, grip cone, and lift slider
            if(gamepad1.left_bumper) {
                autoLoadCone();
                // set arm, claw, slider position after grep.
                armServoPosition = armServo.getPosition();
                clawServoPosition = clawServo.getPosition();
                sliderMotorTargetPosition = SliderMotor.getCurrentPosition();
            }

            //  auto driving, unload cone
            if(gamepad1.right_bumper) {
                autoUnloadCone();
                // set arm, claw, slider position after grep.
                armServoPosition = armServo.getPosition();
                clawServoPosition = clawServo.getPosition();
                sliderMotorTargetPosition = SliderMotor.getCurrentPosition();
            }
        }

        // The motor stop on their own but power is still applied. Turn off motor.
        SliderMotor.setPower(0.0);
    }

    /* 1. Robot moving back to aim at junction for unloading cone
     * 2. Slider moving down a little bit to put cone in junction pole
     * 3. Open claw to fall down cone
     * 4. Lift slider from junction pole
     * 5. Robot moving back to leave junction
     * 6. Slider moving down to get ready to grip another cone
     */
    private void autoUnloadCone() {
        armServo.setPosition(ARM_UNLOAD_POSITION);
        robotMovingDistance(-robotAutoUnloadMovingDistance, true); // moving back in feet

        // move down slider a little bit (100 count) to unload cone
        sliderMotorTargetPosition = SliderMotor.getCurrentPosition();
        SliderMotor.setTargetPosition(sliderMotorTargetPosition - SLIDER_MOVE_DOWN_POSITION);
        waitMotorActionComplete(SliderMotor);

        clawServo.setPosition(CLAW_OPEN_POS); // unload  cone
        sleep(400); // wait 0.4 sec to make sure clawServo is at grep position

        SliderMotor.setTargetPosition(sliderMotorTargetPosition);
        waitMotorActionComplete(SliderMotor);
        robotMovingDistance(-robotAutoUnloadMovingDistance, true); // move out from junction
        armServo.setPosition(ARM_LOAD_POSITION);
        SliderMotor.setTargetPosition(READY_FOR_GRIP_POSITION);
        waitMotorActionComplete(SliderMotor);
    }

    /* 1. Lift slider and open claw to get read to load a cone
     * 2. Robot moving back to aim at cone for loading
     * 2. Slider moving down to load the cone
     * 3. Close claw to grip the cone
     * 4. Lift slider to low junction position for unloading
     * 5. Robot moving back to leave junction
     * 6. Slider moving down to get ready to grip another cone
     */
    private void autoLoadCone() {
        SliderMotor.setTargetPosition(READY_FOR_GRIP_POSITION);
        clawServo.setPosition(CLAW_OPEN_POS);
        armServo.setPosition(ARM_LOAD_POSITION);
        SliderMotor.setPower(SLIDER_MOTOR_POWER); // slider motor starts movement
        robotMovingDistance(-robotAutoLoadMovingDistance, true); // moving back
        SliderMotor.setTargetPosition(SLIDER_MIN_POS);
        waitMotorActionComplete(SliderMotor);
        clawServo.setPosition(CLAW_CLOSE_POS);
        sleep(400); // wait 0.4 sec to make sure clawServo is at grep position
        armServo.setPosition(ARM_UNLOAD_POSITION);
        SliderMotor.setTargetPosition(LOW_JUNCTION_POS);
        waitMotorActionComplete(SliderMotor);
    }

    private void robotMovingDistance(double targetDistance, boolean isBackForward) {
        int targetPosition = (int)(targetDistance * COUNTS_PER_FEET_MOTION);
        telemetry.addData("Status", "driving target position %d", targetPosition);
        setTargetPositionsToWheels(targetPosition, isBackForward);
        robotWithEncoderModeOn(true); // turn on encoder mode
        setPowerToWheels(ADJUST_POSITION_POWER); // low speed for more accurate, start moving
        waitMotorActionComplete(FrontLeftDrive); // just check one wheel.
        setPowerToWheels(0.0); //stop moving
        robotWithEncoderModeOn(false); // turn off encoder mode
    }

    private void robotWithEncoderModeOn(boolean withEncoder) {
        if (withEncoder) {
            FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
            // set back to WITHOUT ENCODER mode
            FrontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void setTargetPositionsToWheels(int tPos, boolean isBF) {
        if (isBF) {
            FrontLeftDrive.setTargetPosition( tPos );
            FrontRightDrive.setTargetPosition( tPos );
            BackLeftDrive.setTargetPosition( tPos );
            BackRightDrive.setTargetPosition( tPos );
        }
        else // move left or right, positive for right
        {
            FrontLeftDrive.setTargetPosition( tPos );
            FrontRightDrive.setTargetPosition( -tPos );
            BackLeftDrive.setTargetPosition( -tPos );
            BackRightDrive.setTargetPosition( tPos );
        }

    }

    private void setPowerToWheels(double power) {
        FrontLeftDrive.setPower(power);
        FrontRightDrive.setPower(power);
        BackLeftDrive.setPower(power);
        BackRightDrive.setPower(power);
    }

    private void waitMotorActionComplete(DcMotor mot) {
        while(mot.isBusy()) {
            idle();
        }
    }


}