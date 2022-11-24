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

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Reno.poc.ConceptSleeveDetection;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

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

@Autonomous(name="AutonomousRight", group="Concept")
//@Disabled
public class AutonomousRight extends LinearOpMode {

    public static boolean debugFlag = true;

    // Declare OpMode members.
    static final double MAX_WAIT_TIME = 8.0; // in seconds
    private final ElapsedTime runtime = new ElapsedTime();
    private final ChassisWith4Motors chassis = new ChassisWith4Motors();

    // slider position variables
    private final SlidersWith2Motors slider = new SlidersWith2Motors();
    static final int coneStack5th = (int)(SlidersWith2Motors.COUNTS_PER_INCH * 5.2); // the 5th cone position in the cone stack. The lowest cone is the 1th one.
    static final int coneLoadStackGap = (int)(SlidersWith2Motors.COUNTS_PER_INCH *  1.32);
    static final int GROUND_POSITION = 0; // Ground(0 inch)
    static final int WALL_POSITION = (int)(SlidersWith2Motors.COUNTS_PER_INCH * 7.0);  // 7 inch
    static final int MEDIUM_JUNCTION_POS = (int)(SlidersWith2Motors.COUNTS_PER_INCH * 23.5); //23.5 inch
    static final int HIGH_JUNCTION_POS = (int)(SlidersWith2Motors.COUNTS_PER_INCH * 33.5); //33.5 inch
    static final int SLIDER_MOVE_DOWN_POSITION = SlidersWith2Motors.COUNTS_PER_INCH * 3; // move down 6 inch to unload cone

    // claw servo motor variables
    private Servo clawServo = null;
    static final double CLAW_OPEN_POS = 0.31;     // Maximum rotational position
    static final double CLAW_CLOSE_POS = 0.08;
    double clawServoPosition = CLAW_OPEN_POS;

    // arm servo variables, not used in current prototype version.
    private Servo armServo = null;

    // variables for autonomous
    double matCenterToJunctionDistance = 15;
    double robotAutoLoadMovingDistance = 1.0; // in INCH
    double robotAutoUnloadMovingDistance = 3.5; // in INCH
    double backToMatCenterDistance = matCenterToJunctionDistance - robotAutoUnloadMovingDistance - 0.5; // in INCH
    static final double matCenterToConeStack = 27.0; // inch


    // sensors
    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;// best collected within 2cm of the target

    // camera and sleeve color
    ConceptSleeveDetection.ParkingPosition myParkingLot = ConceptSleeveDetection.ParkingPosition.UNKNOWN;
    ConceptSleeveDetection sleeveDetection;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    boolean isCameraInstalled = true;


    // variables for location shift
    double[] xyShift = {0.0, -0.5};


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Logging.log("Status - Initialized");

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
        clawServoPosition = CLAW_CLOSE_POS;
        clawServo.setPosition(clawServoPosition);

        // camera for sleeve color detect
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        sleeveDetection = new ConceptSleeveDetection();

        // Sleeve cone location in the image
        Point sleeveTopLeftPoint = new Point(145, 168);

        // Width and height for the bounding box of sleeve cone
        ConceptSleeveDetection.REGION_WIDTH = 30;
        ConceptSleeveDetection.REGION_HEIGHT = 50;
        ConceptSleeveDetection.SLEEVE_TOPLEFT_ANCHOR_POINT = sleeveTopLeftPoint;

        if (isCameraInstalled) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(
                    WebcamName.class, webcamName), cameraMonitorViewId);

            camera.setPipeline(sleeveDetection);

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        }
        myParkingLot = sleeveDetection.getPosition();

        while (!isStarted()) {
            telemetry.addData("Parking position: ", myParkingLot);
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {

            autonomousCore();

            Logging.log("Autonomous - total Run Time: " + runtime);
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update(); // update message at the end of loop
        }

        // The motor stop on their own but power is still applied. Turn off motor.
        slider.stop();
        chassis.setPowers(0.0);
    }

    /** code for autonomous
     * 1. take a picture, recognize the color on sleeve signal
     * 2. Move robot the high junction
     * 3. Unload cone on high junction
     * 4. Move robot to cone loading area
     * 5. Load cone
     * 6. Move robot to parking area
     */
    private void autonomousCore() {
        double[] backgroundColor = {1.0, 1.0, 1.0}; // ambient color
        double[] sleeveColor = {1.0, 1.0, 1.0}; // cone color
        double parkingLocation; // distance between cone loading area to parking area, in inch

        slider.setPower(SlidersWith2Motors.SLIDER_MOTOR_POWER);

        slider.setPosition(MEDIUM_JUNCTION_POS);
        if (ConceptSleeveDetection.ParkingPosition.UNKNOWN != myParkingLot) {
            //move center of robot to the edge of 3rd mat
            chassis.runToPosition(65.5, true);
        }
        else {
            sleep(500); // wait for preloaded cone to lifted.
            readColorSensor(backgroundColor);
            Logging.log("Autonomous - complete background color read.");
            // drive robot to sleeve cone
            chassis.runToPosition(21.5, true);
            readColorSensor(sleeveColor); // reading sleeve signal
            Logging.log("Autonomous - complete Sleeve color read.");
            // push sleeve cone out, and reading background color for calibration
            chassis.runToPosition(44, true);
        }
        parkingLocation = calculateParkingLocation(sleeveColor, backgroundColor);
        Logging.log("Autonomous - parking lot aisle location: %.2f", parkingLocation);

        // turn robot 0 degrees
        chassis.rotateIMUTargetAngle(0.0);
        

        // lift slider during strafe to high junction
        chassis.runToPosition(-14.5, true); // get rid of sleeve cone, and back to the center of mat
        slider.setPosition(HIGH_JUNCTION_POS);

        // turn robot 45 degrees left
        chassis.rotateIMUTargetAngle(45.0);
   
        slider.waitRunningComplete();

        //drive forward and let V to touch junction
        chassis.runToPosition(matCenterToJunctionDistance, true);
        Logging.log("Autonomous - Robot V reached junction.");
        
        // drop cone and back to the center of mat
        autoUnloadCone(backToMatCenterDistance);

        for(int autoLoop = 0; autoLoop < 2; autoLoop++) {
            Logging.log("Autonomous - loop index: %d ", autoLoop);

            slider.setPosition(WALL_POSITION);

            // right turn 135 degree
            chassis.rotateIMUTargetAngle(-90.0);

            chassis.locationShiftCalculation(xyShift);
            // strafe to the left a little bit to compensate for the shift from 135 degree rotation(currently 1 inch).
            chassis.runToPosition(xyShift[0], false);

            sleep(100); // wait for chassis stop from previous rotation inertia.

            // adjust position and double rotation for accurate 135
            chassis.rotateIMUTargetAngle(-90.0);

            // drive robot to loading area. xyShift is according to testing from 135 degrees turning.
            chassis.runToPosition(matCenterToConeStack - xyShift[1], true);
            Logging.log("Autonomous - Robot has arrived loading area.");

            // load cone
            autoLoadCone(coneStack5th - coneLoadStackGap * autoLoop);

            // lift cone
            slider.setPosition(WALL_POSITION);

            // make sure robot is still in the same orientation before back to junction
            chassis.rotateIMUTargetAngle(-90.0);

            slider.waitRunningComplete(); // make sure slider has been lifted before moving out cone stack.

            // lift slider during rotation.
            slider.setPosition(MEDIUM_JUNCTION_POS);

            // drive back to high junction. 1.0 inch has been moved back during autoLoadCone()
            chassis.runToPosition(-(matCenterToConeStack - 1.0), true);
            Logging.log("Autonomous - Robot arrived the mat center near high junction.");

            // lift slider during rotation.
            slider.setPosition(HIGH_JUNCTION_POS);

            // left turn 135 degree facing to high junction
            chassis.rotateIMUTargetAngle(45.0);

            sleep(100); // wait for chassis stop from previous rotation inertia.

            chassis.rotateIMUTargetAngle(45.0);

            chassis.locationShiftCalculation(xyShift);
            chassis.runToPosition(-xyShift[0], false);

            slider.waitRunningComplete(); // make sure slider has been lifted
            Logging.log("Autonomous - slider is positioned to high junction.");

            // moving forward V to junction
            chassis.runToPosition(matCenterToJunctionDistance + xyShift[1], true);

            // Make sure it is 45 degree before V leaving junction
            chassis.rotateIMUTargetAngle(45.0);

            sleep(100); // avoid junction shaking
            // unload cone & adjust
            autoUnloadCone(backToMatCenterDistance - 0.5); // 0.5 is for the cone has been in junction
            Logging.log("Autonomous - cone %d has been unloaded.", autoLoop + 2);

            // lower slider and get ready to load cone
            slider.setPosition(WALL_POSITION);
        }
        Logging.log("Autonomous -  last cone has been unloaded.");

        //rotate 45 degrees to keep orientation at 90
        chassis.rotateIMUTargetAngle(90.0);

        // lower slider in prep for tele-op
        slider.setPosition(GROUND_POSITION);

        // drive to final parking lot
        chassis.runToPosition(-parkingLocation, true); // strafe robot to parking
        Logging.log("Autonomous - Arrived at parking lot aisle: %.2f", parkingLocation);

        slider.waitRunningComplete();
        Logging.log("Autonomous - slider lowered to ground.");
        Logging.log("Autonomous - Autonomous complete.");
    }

    /**
     * Calculate the destination parking area according to sleeve color.
     * @param s three ratios values of sleeve signal color reading from color sensor
     * @param b three ratios values of background color reading from color sensor
     * return the distance between cone stack and parking area, in inch.
     */
    private double calculateParkingLocation(@NonNull double[] s, @NonNull double[] b) {
        int channel = 0;
        double location;
        String color;

        if(ConceptSleeveDetection.ParkingPosition.UNKNOWN != myParkingLot) { // camera
            switch (myParkingLot) {
                case LEFT: // red
                    location = -2.0 * 12; // parking lot #1 (red), first mat
                    color = "red";
                    break;
                case CENTER: // green
                    location = 0.0; // parking lot #2 (green), second mat
                    color = "green";
                    break;
                case RIGHT: // blue
                    location = 2.0 * 12; // parking lot #3 (blue), third mat
                    color = "blue";
                    break;
                default:
                    location = 0.0;
                    color = "Unknown";
            }
            Logging.log("Autonomous - Sleeve color from camera is %s", color);
        }
        else { // color sensor
            if (0 == b[0]) return 0.0;
            if (0 == b[1]) return 0.0;
            if (0 == b[2]) return 0.0;
            double[] ratio = {s[0] / b[0], s[1] / b[1], s[2] / b[2]};

            // find the maximum value in ratio[]
            double maxV = ratio[0];
            for (int i = 1; i < 3; i++) {
                if (ratio[i] > maxV) {
                    channel = i;
                    maxV = ratio[i];
                }
            }
            switch (channel) {
                case 0: // red
                    location = -2.0 * 12; // parking lot #1 (red), third mat
                    color = "red";
                    break;
                case 1: // green
                    location = 0; // parking lot #2 (green), third mat
                    color = "green";
                    break;
                case 2: // blue
                    location = 2.0 * 12; // parking lot #3 (blue), third mat
                    color = "blue";
                    break;
                default:
                    location = 0.0;
                    color = "Unknown";
            }
            Logging.log("Autonomous - channel = %d, max value = %.3f", channel, maxV);
            Logging.log("Autonomous - Sleeve color from color sensor is %s", color);
            telemetry.addData("color", "rgb ratio, %.2f, %.2f, %.2f", ratio[0], ratio[1], ratio[2]);
        }

        return location;
    }

    /**
     * Read color from color sensor and translate three values to relative ratio.
     * @param colorRatio three ratios values of color reading from color sensor
     */
    private void readColorSensor(@NonNull double[] colorRatio ) {
        //color sensor control
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        int total = r + g + b;
        if (0 != total) {
            colorRatio[0] = (double) r / total;
            colorRatio[1] = (double) g / total;
            colorRatio[2] = (double) b / total;
        }
        telemetry.addLine()
                .addData("Red  ", colorSensor.red())
                .addData("Green", colorSensor.green())
                .addData("Blue ", colorSensor.blue());
        telemetry.addData("color", "rgb ratio, %.2f, %.2f, %.2f", colorRatio[0], colorRatio[1], colorRatio[2]);
        Logging.log("Autonomous - Red: %d, green: %d, blue: %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        Logging.log("Autonomous - Red: %.2f, green: %.2f, blue: %.2f", colorRatio[0], colorRatio[1], colorRatio[2]);
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
    private void autoLoadCone(int coneLocation) {
        clawServo.setPosition(CLAW_OPEN_POS);
        slider.setPosition(coneLocation);
        chassis.runToPosition(-robotAutoLoadMovingDistance, true); // back a little bit to avoid stuck.
        slider.waitRunningComplete();
        clawServo.setPosition(CLAW_CLOSE_POS);
        Logging.log("Auto load - Cone has been loaded.");
        sleep(200); // wait to make sure clawServo is at grep position
    }

    /**
     * 1. Robot moving back to aim at junction for unloading cone
     * 2. Slider moving down a little bit to put cone in junction pole
     * 3. Open claw to fall down cone
     * 4. Lift slider from junction pole during the robot moving back to leave junction
     * 6. Slider moving down to get ready to grip another cone
     * @param backDistanceAfterUnloading: the moving distance after unloading the cone.
     */
    private void autoUnloadCone(double backDistanceAfterUnloading) {
        // moving back in inch
        chassis.runToPosition(-robotAutoUnloadMovingDistance, true);

        // move down slider a little bit to unload cone
        int sliderTargetPosition = slider.getPosition();
        int moveSlider = sliderTargetPosition - SLIDER_MOVE_DOWN_POSITION;
        moveSlider = Math.max(moveSlider, SlidersWith2Motors.SLIDER_MIN_POS);
        slider.setPosition(moveSlider);
        slider.waitRunningComplete();

        clawServo.setPosition(CLAW_OPEN_POS); // unload cone
        sleep(100); // make sure cone has been unloaded
        slider.setPosition(sliderTargetPosition);
        chassis.runToPosition(-backDistanceAfterUnloading, true); // move out from junction
        slider.setPosition(WALL_POSITION);
        Logging.log("Auto unload - Cone has been unloaded.");
    }

}

