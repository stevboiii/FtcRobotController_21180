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

 /*
  * PID controller and IMU codes are copied from
  * https://stemrobotics.cs.pdx.edu/node/7268%3Froot=4196.html
  */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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

@TeleOp(name="TeleopDualDrivers", group="Concept")
//@Disabled
public class TeleopDualDrivers extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // chassis
    private final ChassisWith4Motors chassis = new ChassisWith4Motors();

    // slider
    private final SlidersWith2Motors slider = new SlidersWith2Motors();

    // Driving motor variables
    static final double HIGH_SPEED_POWER = 0.6;
    static final double SLOW_DOWN_POWER = 0.2;

    // slider motor variables
    static final double SLIDER_MOTOR_POWER = 0.8;
    static final int COUNTS_PER_INCH = SlidersWith2Motors.COUNTS_PER_INCH;
    static final int FOUR_STAGE_SLIDER_MAX_POS = 4200;  // with 312 RPM motor.
    static final int SLIDER_MIN_POS = 0;
    static final int WALL_POSITION = (int)(COUNTS_PER_INCH * 7.0);
    static final int LOW_JUNCTION_POS = (int)(COUNTS_PER_INCH * 13.5); // 13.5 inch
    static final int MEDIUM_JUNCTION_POS = (int)(COUNTS_PER_INCH * 23.5);
    static final int HIGH_JUNCTION_POS = (int)(COUNTS_PER_INCH * 33.5);
    static final int SLIDER_MOVE_DOWN_POSITION = COUNTS_PER_INCH * 3; // move down 3 inch to unload cone
    static final int POSITION_COUNTS_FOR_ONE_REVOLUTION = 538; // for 312 rpm motor
    int motorPositionInc = POSITION_COUNTS_FOR_ONE_REVOLUTION / 4;
    int sliderTargetPosition = 0;

    // claw servo motor variables
    private Servo clawServo = null;
    static final double CLAW_INCREMENT = -0.24;  // amount to slew servo each CYCLE_MS cycle
    static final double CLAW_OPEN_POS = 0.31;
    static final double CLAW_CLOSE_POS = 0.08;
    static final double CLAW_MAX_POS = CLAW_OPEN_POS; // Maximum rotational position
    static final double CLAW_MIN_POS = CLAW_CLOSE_POS;  // Minimum rotational position
    double clawServoPosition = CLAW_OPEN_POS;


    // arm servo variables, not used in current prototype version.
    private Servo armServo = null;

    // variables for auto load and unload cone
    double robotAutoLoadMovingDistance = 1.0; // in INCH
    double robotAutoUnloadMovingDistance = 3.5; // in INCH

    // sensors
    private DistanceSensor distanceSensor;
    static final double CLOSE_DISTANCE = 8.0; // the distance to slow down robot driving
    private ColorSensor colorSensor;// best collected the color within 2cm of the target

    // debug flags, turn it off for formal version to save time of logging
    boolean debugFlag = true;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        slider.init(hardwareMap, "RightSlider", "LeftSlider");
        chassis.init(hardwareMap, "FrontLeft", "FrontRight",
                "BackLeft", "BackRight");

        armServo = hardwareMap.get(Servo.class, "ArmServo");
        clawServo = hardwareMap.get(Servo.class, "ClawServo");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        // claw servo motor initial
        clawServoPosition = CLAW_OPEN_POS;
        clawServo.setPosition(clawServoPosition);

        // sensors
        boolean distanceSensorEnabled = false;

        //game pad setting
        float robotMovingBackForth;
        float robotMovingRightLeft;
        float robotTurn;
        float sliderUpDown;
        boolean sliderGroundPsition;
        boolean sliderWallPosition;
        boolean sliderLowJunctionPosition;
        boolean sliderMediumJunctionPosition;
        boolean sliderHighJunctionPosition;
        boolean sliderSkipLimitation = false;
        boolean sliderResetPosition = false;
        boolean clawClose;
        boolean clawOpen;
        boolean autoLoadConeOn;
        boolean autoUnloadConeOn;
        boolean distanceSensorOn;

        boolean dualDriverMode = true;
        Gamepad myGamePad;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Game pad buttons design
            if (gamepad1.start && (gamepad1.left_trigger > 0)) {
                dualDriverMode = false;
            }

            if (gamepad1.start && (gamepad1.right_trigger > 0)) {
                dualDriverMode = true;
            }

            if (dualDriverMode) {
                myGamePad = gamepad2;
            }
            else {
                myGamePad = gamepad1;
            }

            //gamepad1 buttons
            robotMovingBackForth = gamepad1.left_stick_y;
            robotMovingRightLeft = gamepad1.left_stick_x;
            robotTurn            = gamepad1.right_stick_x;
            distanceSensorOn     = gamepad1.back;
            autoLoadConeOn       = gamepad1.left_bumper;
            autoUnloadConeOn     = gamepad1.right_bumper;

            // gamepad1(single driver) or gamepad2(dual driver) buttons
            sliderUpDown                = myGamePad.right_stick_y;
            sliderWallPosition          = myGamePad.x;
            sliderLowJunctionPosition   = myGamePad.a;
            sliderMediumJunctionPosition= myGamePad.b;
            sliderHighJunctionPosition  = myGamePad.y;
            clawClose                   = myGamePad.dpad_up;
            clawOpen                    = myGamePad.dpad_down;

            sliderGroundPsition         = myGamePad.dpad_left || myGamePad.dpad_right;

            if (dualDriverMode)
            {
                sliderSkipLimitation = gamepad2.left_bumper;
                sliderResetPosition = (gamepad2.right_bumper && gamepad2.left_bumper);
            }

            // sensors
            if (distanceSensorOn) {
                distanceSensorEnabled = !distanceSensorEnabled;
            }

            // Setup a variable for each drive wheel to save power level for telemetry
            double maxDrivePower = HIGH_SPEED_POWER;

            //distance sensor control
            if ((distanceSensor.getDistance(DistanceUnit.INCH) < CLOSE_DISTANCE) &&
                    distanceSensorEnabled) {
                maxDrivePower = SLOW_DOWN_POWER;
            }

            double drive = maxDrivePower * robotMovingBackForth;
            double turn  =  maxDrivePower * (-robotTurn);
            double strafe = maxDrivePower * (-robotMovingRightLeft);

            chassis.drivingWithPID(drive, turn, strafe);

            // use Y button to lift up the slider reaching high junction
            if (sliderHighJunctionPosition) {
                sliderTargetPosition = HIGH_JUNCTION_POS;
            }

            // use B button to lift up the slider reaching medium junction
            if (sliderMediumJunctionPosition) {
                sliderTargetPosition = MEDIUM_JUNCTION_POS;
            }

            // use A button to lift up the slider reaching low junction
            if (sliderLowJunctionPosition) {
                sliderTargetPosition = LOW_JUNCTION_POS;
            }

            // use X button to move the slider for wall position
            if (sliderWallPosition) {
                sliderTargetPosition = WALL_POSITION;
            }

            if (sliderGroundPsition) {
                sliderTargetPosition = SLIDER_MIN_POS;
            }

            // use right stick_Y to lift or down slider continuously
            sliderTargetPosition -= (int)((sliderUpDown) * motorPositionInc);
            if (!sliderSkipLimitation) {
                sliderTargetPosition = Range.clip(sliderTargetPosition, SLIDER_MIN_POS,
                        FOUR_STAGE_SLIDER_MAX_POS);
            }

            if (sliderResetPosition) {
                slider.resetEncoders();
                sliderTargetPosition = SLIDER_MIN_POS;
            }
            slider.setPosition(sliderTargetPosition);
            slider.setPower(SLIDER_MOTOR_POWER); // slider motor start movement

            // Keep stepping up until we hit the max value.
            if (clawClose) {
                clawServoPosition += CLAW_INCREMENT;
            }

            if (clawOpen) {
                clawServoPosition -= CLAW_INCREMENT;
            }
            clawServoPosition = Range.clip(clawServoPosition, CLAW_MIN_POS, CLAW_MAX_POS);
            clawServo.setPosition(clawServoPosition);

            //  auto driving, grip cone, and lift slider
            if(autoLoadConeOn) {
                loadCone(SLIDER_MIN_POS); // Always on ground during teleop mode
            }

            //  auto driving, unload cone
            if(autoUnloadConeOn) {
                unloadCone();
            }

            if (debugFlag) {
                // config log
                telemetry.addData("distance sensor: ", distanceSensorEnabled? "On" : "Off");
                telemetry.addData("Dual driver mode: ", dualDriverMode? "On" : "Off");

                // imu log
                telemetry.addData("imu heading ","%.2f", chassis.lastAngles.firstAngle);
                telemetry.addData("global heading ", "%.2f", chassis.globalAngle);
                telemetry.addData("Correction  ", "%.2f", chassis.correction);

                // sensor log
                telemetry.addData("Distance sensor = ", "%.2f", distanceSensor.getDistance(DistanceUnit.INCH));

                // claw servo log
                telemetry.addData("Status", "Claw Servo position %.2f", clawServoPosition);

                // slider motors log
                telemetry.addData("Status", "slider motor Target position %d",
                        sliderTargetPosition);
                telemetry.addData("Status", "Right slider motor current position %d",
                        slider.RightSliderMotor.getCurrentPosition());
                telemetry.addData("Status", "Left slider motor current position %d",
                        slider.LeftSliderMotor.getCurrentPosition());

                // drive motors log
                telemetry.addData("Max driving power ", "%.2f", maxDrivePower);

                // running time
                telemetry.addData("Status", "Run Time: " + runtime);
            }
            telemetry.update(); // update message at the end of while loop
        }

        // The motor stop on their own but power is still applied. Turn off motor.
        slider.stop();
        chassis.setPowers(0.0);
    }

    /**
     * 1. Robot moving back to aim at junction for unloading cone
     * 2. Slider moving down a little bit to put cone in junction pole
     * 3. Open claw to fall down cone
     * 4. Lift slider from junction pole
     * 5. Robot moving back to leave junction
     * 6. Slider moving down to get ready to grip another cone
     */
    private void unloadCone() {
        chassis.runToPosition(-robotAutoUnloadMovingDistance, true); // moving back in inch

        // move down slider a little bit to unload cone
        sliderTargetPosition = slider.getPosition();
        int moveSlider = sliderTargetPosition - SLIDER_MOVE_DOWN_POSITION;
        moveSlider = Math.max(moveSlider, SLIDER_MIN_POS);
        slider.setPosition(moveSlider);
        slider.waitRunningComplete();

        clawServo.setPosition(CLAW_OPEN_POS); // unload  cone
        sleep(100); // to make sure clawServo is at open position
        slider.setPosition(sliderTargetPosition);
        clawServoPosition = CLAW_OPEN_POS; // keep claw position
        chassis.runToPosition(-robotAutoUnloadMovingDistance, true); // move out from junction
        sliderTargetPosition = WALL_POSITION;
    }

    /**
     * During autonomous, cone may be located with different height position
     * 1. Lift slider and open claw to get read to load a cone
     * 2. Robot moving back to aim at cone for loading
     * 2. Slider moving down to load the cone
     * 3. Close claw to grip the cone
     * 4. Lift slider to low junction position for unloading
     * 5. Robot moving back to leave junction
     * 6. Slider moving down to get ready to grip another cone
     * @param coneLocation: the target cone high location.
     */
    private void loadCone(int coneLocation) {
        clawServo.setPosition(CLAW_OPEN_POS);
        slider.setPosition(coneLocation);
        chassis.runToPosition(-robotAutoLoadMovingDistance, true); // moving to loading position
        slider.waitRunningComplete();
        clawServo.setPosition(CLAW_CLOSE_POS);
        Logging.log("Auto load - Cone has been loaded.");
        sleep(200); // wait to make sure clawServo is at grep position
        clawServoPosition = CLAW_CLOSE_POS; // keep claw position
        sliderTargetPosition = LOW_JUNCTION_POS;
    }

}