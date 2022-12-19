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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.List;

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

@TeleOp(name="Teleop_DualDrivers", group="Concept")
//@Disabled
public class TeleopDualDrivers extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // chassis
    private final ChassisWith4Motors chassis = new ChassisWith4Motors();

    // Driving motor variables
    static final double HIGH_SPEED_POWER = 0.6;

    // slider motor power variables
    private final SlidersWith2Motors slider = new SlidersWith2Motors();
    static final double SLIDER_MOTOR_POWER = 0.7;

    // slider position variables
    final int FOUR_STAGE_SLIDER_MAX_POS = 4200;  // with 312 RPM motor.
    final int SLIDER_MIN_POS = 0;
    final int GROUND_CONE_POSITION = slider.COUNTS_PER_INCH; // 1 inch
    final int coneLoadStackGap = (int)(slider.COUNTS_PER_INCH *  1.2);
    final int GROUND_JUNCTION_POS = (int)(GROUND_CONE_POSITION + slider.COUNTS_PER_INCH);
    final int WALL_POSITION = (int)(slider.COUNTS_PER_INCH * 7.5);  // 7.5 inch
    final int MEDIUM_JUNCTION_POS = (int)(slider.COUNTS_PER_INCH * 24.5); //23.5 inch
    final int HIGH_JUNCTION_POS = (int)(slider.COUNTS_PER_INCH * 34.5); //33.5 inch
    final int SLIDER_MOVE_DOWN_POSITION = slider.COUNTS_PER_INCH * 4; // move down 3 inch to unload cone
    final int LOW_JUNCTION_POS = (int)(slider.COUNTS_PER_INCH * 14.7); // 13.5 inch
    final int POSITION_COUNTS_FOR_ONE_REVOLUTION = 538; // for 312 rpm motor
    int motorPositionInc = POSITION_COUNTS_FOR_ONE_REVOLUTION / 10;
    int sliderTargetPosition = 0;

    // claw servo motor variables
    private Servo clawServo = null;
    final double CLAW_INCREMENT = -0.24;  // amount to slew servo each CYCLE_MS cycle
    final double CLAW_OPEN_POS = 0.31;
    final double CLAW_CLOSE_POS = 0.08;
    final double CLAW_MAX_POS = CLAW_OPEN_POS; // Maximum rotational position
    final double CLAW_MIN_POS = CLAW_CLOSE_POS;  // Minimum rotational position
    double clawServoPosition = CLAW_OPEN_POS;

    // arm servo variables, not used in current prototype version.
    private Servo armServo = null;

    // variables for auto load and unload cone
    double robotAutoLoadMovingDistance = 1.0; // in INCH
    double robotAutoUnloadMovingDistance = 3.5; // in INCH

    // debug flags, turn it off for formal version to save time of logging
    boolean debugFlag = true;

    // voltage management
    LynxModule ctrlHub;
    LynxModule exHub;
    private final double powerRampRate = 0.3/100; // 0.2 per 100 ms


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        GamePadButtons gpButtons = new GamePadButtons();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        slider.init(hardwareMap, "RightSlider", "LeftSlider");
        chassis.init(hardwareMap, "FrontLeft", "FrontRight",
                "BackLeft", "BackRight");

        armServo = hardwareMap.get(Servo.class, "ArmServo");
        clawServo = hardwareMap.get(Servo.class, "ClawServo");

        // power control
        ctrlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        exHub = hardwareMap.get(LynxModule.class, "Control Hub");
        double ctrlHubCurrent, ctrlHubVolt, exHubCurrent, exHubVolt, auVolt;
        double maxCtrlCurrent = 0.0, minCtrlVolt = 15.0, maxExCurrent = 0.0, minExVolt = 15.0, minAuVolt = 15.0;

        // claw servo motor initial
        clawServoPosition = CLAW_OPEN_POS;
        clawServo.setPosition(clawServoPosition);

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double timeStamp = 0.0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //gamepad1 buttons
            gpButtons.checkGamepadButtons(gamepad1, gamepad2);

            double maxDrivePower;
            double chassisCurrentPower = chassis.getAveragePower();
            double deltaTime = runtime.milliseconds() - timeStamp;
            double maxP = chassisCurrentPower + powerRampRate * deltaTime;
            timeStamp = runtime.milliseconds();

            maxDrivePower = Math.min(maxP, HIGH_SPEED_POWER);

            double drive = maxDrivePower * gpButtons.robotDrive;
            double turn  =  maxDrivePower * (-gpButtons.robotTurn);
            double strafe = maxDrivePower * (-gpButtons.robotStrafe);

            chassis.drivingWithPID(drive, turn, strafe, true);

            // use Y button to lift up the slider reaching high junction
            if (gpButtons.sliderHighJunction) {
                sliderTargetPosition = HIGH_JUNCTION_POS;
                slider.setPosition(sliderTargetPosition);
                slider.setPower(SLIDER_MOTOR_POWER);
            }

            // use B button to lift up the slider reaching medium junction
            if (gpButtons.sliderMediumJunction) {
                sliderTargetPosition = MEDIUM_JUNCTION_POS;
                slider.setPosition(sliderTargetPosition);
                slider.setPower(SLIDER_MOTOR_POWER);
            }

            // use A button to lift up the slider reaching low junction
            if (gpButtons.sliderLowJunction) {
                sliderTargetPosition = LOW_JUNCTION_POS;
                slider.setPosition(sliderTargetPosition);
                slider.setPower(SLIDER_MOTOR_POWER);
            }

            // use X button to move the slider for wall position
            if (gpButtons.sliderWallPosition) {
                sliderTargetPosition = WALL_POSITION;
                slider.setPosition(sliderTargetPosition);
                slider.setPower(SLIDER_MOTOR_POWER);
            }
            // use dpad left to get to the ground junction position
            if (gpButtons.sliderGroundJunction) {
                sliderTargetPosition = GROUND_JUNCTION_POS;
                slider.setPosition(sliderTargetPosition);
                slider.setPower(SLIDER_MOTOR_POWER);
            }

            // use right stick_Y to lift or down slider continuously
            if (Math.abs(gpButtons.sliderUpDown) > 0) {
                sliderTargetPosition -= (int) ((gpButtons.sliderUpDown) * motorPositionInc);
                if (!gpButtons.sliderSkipLimitation) {
                    sliderTargetPosition = Range.clip(sliderTargetPosition, SLIDER_MIN_POS,
                            FOUR_STAGE_SLIDER_MAX_POS);
                }
                slider.setPosition(sliderTargetPosition);
                slider.setPower(SLIDER_MOTOR_POWER);
            }

            if (gpButtons.sliderResetEncoder) {
                slider.resetEncoders();
                sliderTargetPosition = SLIDER_MIN_POS;
                slider.setPosition(sliderTargetPosition);
                slider.setPower(SLIDER_MOTOR_POWER);
            }

            // Set position only when button is hit.
            if (gpButtons.clawClose) {
                clawServoPosition += CLAW_INCREMENT;
                clawServoPosition = Range.clip(clawServoPosition, CLAW_MIN_POS, CLAW_MAX_POS);
                clawServo.setPosition(clawServoPosition);
            }

            // Set position only when button is hit.
            if (gpButtons.clawOpen) {
                clawServoPosition -= CLAW_INCREMENT;
                clawServoPosition = Range.clip(clawServoPosition, CLAW_MIN_POS, CLAW_MAX_POS);
                clawServo.setPosition(clawServoPosition);
            }

            //  auto driving, grip cone, and lift slider
            if(gpButtons.autoLoadGroundCone) {
                loadCone(GROUND_CONE_POSITION - 40); // Always on ground during teleop mode
            }

            //  auto driving, grip cone, and lift slider
            if(gpButtons.autoLoad34thConeStack) {
                loadCone(GROUND_CONE_POSITION + coneLoadStackGap * 2); // Always on ground during teleop mode
            }

            //  auto driving, grip cone, and lift slider
            if(gpButtons.autoLoad5thConeStack) {
                loadCone(GROUND_CONE_POSITION + coneLoadStackGap * 3); // Always on ground during teleop mode
            }

            //  auto driving, unload cone
            if(gpButtons.autoUnloadCone) {
                unloadCone();
            }

            if (debugFlag) {
                ctrlHubCurrent = ctrlHub.getCurrent(CurrentUnit.AMPS);
                ctrlHubVolt = ctrlHub.getInputVoltage(VoltageUnit.VOLTS);
                exHubCurrent = exHub.getCurrent(CurrentUnit.AMPS);
                exHubVolt = exHub.getInputVoltage(VoltageUnit.VOLTS);

                auVolt = ctrlHub.getAuxiliaryVoltage(VoltageUnit.VOLTS);
                minAuVolt = Math.min(auVolt, minAuVolt);

                maxCtrlCurrent = Math.max(ctrlHubCurrent, maxCtrlCurrent);
                maxExCurrent = Math.max(exHubCurrent, maxExCurrent);
                minCtrlVolt = Math.min(ctrlHubVolt, minCtrlVolt);
                minExVolt = Math.min(exHubVolt, minExVolt);

                telemetry.addData("Front Center distance sensor", "%.2f", chassis.getFcDSValue());

                telemetry.addData("Max Ctrl hub current = ", "%.2f", maxCtrlCurrent);
                telemetry.addData("Min Ctrl hub Volt = ", "%.2f", minCtrlVolt);
                telemetry.addData("Max Extend hub current = ", "%.2f", maxExCurrent);
                telemetry.addData("Min Extend hub Volt = ", "%.2f", minExVolt);

                Logging.log("Get drive power = %.2f, set drive power = %.2f", chassisCurrentPower, maxP);
                Logging.log("Ctrl hub current = %.2f, max = %.2f", ctrlHubCurrent, maxCtrlCurrent);
                Logging.log("Ctrl hub volt = %.2f, min = %.2f", ctrlHubVolt, minCtrlVolt);
                Logging.log("Extend hub current = %.2f, max = %.2f", exHubCurrent, maxExCurrent);
                Logging.log("Extend hub volt = %.2f, min = %.2f", exHubVolt, minExVolt);
                Logging.log("Auxiliary voltage = %.2f, min = %.2f", auVolt, minAuVolt);

                // imu log
                telemetry.addData("imu heading ", "%.2f", chassis.lastAngles.firstAngle);
                telemetry.addData("global heading ", "%.2f", chassis.globalAngle);
                telemetry.addData("Correction  ", "%.2f", chassis.correction);

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
            }
            // running time
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Status", "While loop Time in ms = ", "%.1f", deltaTime);
            telemetry.update(); // update message at the end of while loop
            Logging.log("While loop time in ms = %.1f.", deltaTime);
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
        moveSlider = Math.max(moveSlider, GROUND_CONE_POSITION);
        slider.setPower(SLIDER_MOTOR_POWER);
        slider.setPosition(moveSlider);
        slider.waitRunningComplete();

        clawServo.setPosition(CLAW_OPEN_POS); // unload  cone
        sleep(100); // to make sure clawServo is at open position
        slider.setPosition(sliderTargetPosition);
        clawServoPosition = CLAW_OPEN_POS; // keep claw position
        chassis.runToPosition(-robotAutoUnloadMovingDistance, true); // move out from junction
        sliderTargetPosition = WALL_POSITION;
        slider.setPosition(sliderTargetPosition);
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
        slider.setPower(SLIDER_MOTOR_POWER);
        clawServo.setPosition(CLAW_OPEN_POS);
        slider.setPosition(coneLocation);
        chassis.runToPosition(-robotAutoLoadMovingDistance, true); // moving to loading position
        slider.waitRunningComplete();
        clawServo.setPosition(CLAW_CLOSE_POS);
        Logging.log("Auto load - Cone has been loaded.");
        sleep(200); // wait to make sure clawServo is at grep position
        clawServoPosition = CLAW_CLOSE_POS; // keep claw position
        sliderTargetPosition = LOW_JUNCTION_POS;
        slider.setPosition(sliderTargetPosition);
    }

}