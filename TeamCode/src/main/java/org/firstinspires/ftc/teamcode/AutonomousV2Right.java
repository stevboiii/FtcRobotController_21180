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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

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

@Autonomous(name="Autonomous V2 Right", group="Concept")
//@Disabled
public class AutonomousV2Right extends AutonomousRight {


    /** code for autonomous
     * 1. take a picture, recognize the color on sleeve signal
     * 2. Move robot the high junction
     * 3. Unload cone on high junction
     * 4. Move robot to cone loading area
     * 5. Load cone
     * 6. Move robot to parking area
     */
    @Override
    public void autonomousCore() {

        // slider setting
        slider.setPower(slider.SLIDER_MOTOR_POWER);
        slider.setPosition(FieldParams.HIGH_JUNCTION_POS);

        //move center of robot to the center of 3rd mat near high junction
        chassis.runToPosition(FieldParams.INIT_POSITION_TO_MAT_CENTER * autonomousStartLocation, false);

        // turn robot to make sure it is at 0 degree before backing to high junction
        sleep(100);
        chassis.rotateIMUTargetAngle(0.0);

        // turn arm to side to get ready for dropping cone
        if (autonomousStartLocation > 0) {
            armClaw.armTurnLeft();
        }
        else {
            armClaw.armTurnRight();
        }

        // driving back to high junction
        chassis.runToPosition(FieldParams.HALF_MAT - FieldParams.ARM_LOCATION_BIAS, true);

        slider.waitRunningComplete();

        // drop cone and back to the center of mat
        autoUnloadCone();

        for(int autoLoop = 0; autoLoop < 4; autoLoop++) {
            Logging.log("Autonomous - loop index: %d ", autoLoop);

            // drive robot to loading area.
            chassis.runToPosition(-FieldParams.HIGH_JUNCTION_TO_CONE_STACK, true);
            Logging.log("Autonomous - Robot has arrived loading area.");

            Logging.log("fcDistance sensor value before loading: %.2f ", chassis.getFcDSValue());
            // load cone
            autoLoadCone(FieldParams.coneStack5th - FieldParams.coneLoadStackGap * autoLoop);

            // lift slider during driving back to mat center.
            slider.setPosition(FieldParams.HIGH_JUNCTION_POS);
            chassis.runToJunction(FieldParams.HIGH_JUNCTION_TO_CONE_STACK - 6.0, 6.0, 20);
            Logging.log("Autonomous - Robot arrived the high junction.");

            slider.waitRunningComplete();
            Logging.log("Autonomous - slider is positioned to high junction.");

            // turn arm to side to get ready for dropping cone
            if (autonomousStartLocation > 0) {
                armClaw.armTurnLeft();
            }
            else {
                armClaw.armTurnRight();
            }
            sleep(100); // wait arm action complete

            // drop cone and back to the center of mat
            autoUnloadCone();
            Logging.log("Autonomous - cone %d has been unloaded.", autoLoop + 2);
        }

        // lower slider in prep for tele-op
        slider.setPosition(FieldParams.GROUND_CONE_POSITION);

        // drive to final parking lot
        chassis.runToPosition(-(parkingLotDis * autonomousStartLocation + FieldParams.HALF_MAT - FieldParams.ARM_LOCATION_BIAS), true);
        Logging.log("Autonomous - Arrived at parking lot Mat: %.2f", parkingLotDis);

        slider.waitRunningComplete();
        Logging.log("Autonomous - Autonomous complete.");
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
    private void autoLoadCone(double coneLocation) {
        armClaw.clawOpen();
        slider.setPosition(coneLocation);
        chassis.runToPosition(robotAutoLoadMovingDistance, true); // back a little bit to avoid stuck.
        slider.waitRunningComplete();
        armClaw.clawClose();
        Logging.log("Auto load - Cone has been loaded.");
        sleep(100); // wait to make sure clawServo is at grep position
        // lift cone and make sure robot is in the same orientation before back to junction
        slider.setPosition(FieldParams.WALL_POSITION);
        // doing angle correction during sleeping, make sure it is 0 degree for next driving
        chassis.rotateIMUTargetAngle(0.0);
        slider.waitRunningComplete(); // make sure slider has been lifted.
    }

    /**
     * Slider moving down a little bit to put cone in junction pole
     * Open claw to fall down cone
     * Lift slider from junction pole during the robot moving back to leave junction
     * Slider moving down to get ready to grip another cone
     */
    private void autoUnloadCone() {
        // move down slider a little bit to unload cone
        double sliderInchPos = slider.getPosition() / (double)slider.COUNTS_PER_INCH;
        slider.setPosition(sliderInchPos - FieldParams.SLIDER_MOVE_DOWN_POSITION);
        slider.waitRunningComplete();
        armClaw.clawOpen();// unload cone
        // doing angle correction during sleeping, make sure it is 0 degree for next driving
        chassis.rotateIMUTargetAngle(0.0);
        sleep(100); // make sure cone has been unloaded
        armClaw.armTurnForward();
        slider.setPosition(FieldParams.WALL_POSITION);
        Logging.log("Auto unload - Cone has been unloaded.");
    }

/**
 * Set robot starting position: 1 for right and -1 for left.
 */
    public void setRobotLocation() {
        autonomousStartLocation = 1;
    }

}