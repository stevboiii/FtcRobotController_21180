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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are case sensitive.
 * Motors type: GoBILDA 312 RPM Yellow Jacket.
 *
 * Motor channel:  Left slider DC motor:        "RightSlider"
 * Motor channel:  Right slider DC motor:        "RightSlider"
 */


public class SlidersWith2Motors
{
    //private
    HardwareMap hardwareMap =  null;
    private final ElapsedTime period  = new ElapsedTime();
    static final double MAX_WAIT_TIME = 8.0; // in seconds

    // slider motor variables
    public DcMotor RightSliderMotor = null;
    public DcMotor LeftSliderMotor = null;
    static final double SLIDER_MOTOR_POWER = 0.9; // save some powers

    // slider position variables
    static final int COUNTS_PER_INCH = 120; // verified by testing.
    static final int FOUR_STAGE_SLIDER_MAX_POS = 4200;  // with 312 RPM motor.
    static final int SLIDER_MIN_POS = 0;

    /**
     * Init slider motors hardware, and set their behaviors.
     * @param hardwareMap the Hardware Mappings.
     */
    public void init(HardwareMap hardwareMap, String rightMotorName, String leftMotorName) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;
        Logging.log("init slider motors.");
        RightSliderMotor = hardwareMap.get(DcMotor.class, rightMotorName);
        LeftSliderMotor = hardwareMap.get(DcMotor.class, leftMotorName);

        /* slider motor control */
        RightSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftSliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset slider motor encoder counts kept by the motor
        resetEncoders();
    }

    /**
     * Set slider motors position.
     * @param sliderMotorPosition the target position for slider left motor and right motor.
     */
    public void setPosition(int sliderMotorPosition) {
        sliderMotorPosition = Range.clip(sliderMotorPosition, SLIDER_MIN_POS, FOUR_STAGE_SLIDER_MAX_POS);
        RightSliderMotor.setTargetPosition(sliderMotorPosition);
        LeftSliderMotor.setTargetPosition(sliderMotorPosition);
    }

    /**
     * Read current slider motors position. Return the mean value of left and right motor positions.
     * return slider motor position.
     */
    public int getPosition() {
        int r = RightSliderMotor.getCurrentPosition();
        int l = LeftSliderMotor.getCurrentPosition();
        return (r+l)/2;
    }

    /**
     * Wait until slider motors complete actions.
     */
    public void waitRunningComplete() {
        double curTime = period.seconds();

        while ((Math.abs(RightSliderMotor.getCurrentPosition() - RightSliderMotor.getTargetPosition()) > COUNTS_PER_INCH * 0.2) &&
                (Math.abs(LeftSliderMotor.getCurrentPosition() - LeftSliderMotor.getTargetPosition()) > COUNTS_PER_INCH * 0.2) &&
                ((period.seconds() - curTime) < MAX_WAIT_TIME)) {
            Thread.yield(); // idle
        }
    }

    /**
     * Set slider motors power to zero.
     */
    public void stop() {
        setPower(0.0);
    }

    /**
     * Set slider motors power.
     * @param p set motors power to p.
     */
    public void setPower(double p) {
        p = Range.clip(p, -1, 1);
        RightSliderMotor.setPower(p);
        LeftSliderMotor.setPower(p);
    }

    /**
     * Reset slider motor encoder counts kept by the motor
     */
    public void resetEncoders() {
        stop();
        RightSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setPosition(0);
        RightSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}

