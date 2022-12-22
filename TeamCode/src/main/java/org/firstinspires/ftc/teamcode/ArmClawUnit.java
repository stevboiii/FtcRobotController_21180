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

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are case sensitive.
 * Motors type: Servo motors for arm and claw.
 *
 * 1. Arm servo motor: ArmServo
 * 2. Claw servo motor: ClawServo
 */
public class ArmClawUnit
{
    //private
    HardwareMap hardwareMap =  null;

    // claw servo motor variables
    private Servo clawServo = null;
    final double CLAW_OPEN_POS = 0.31;
    final double CLAW_CLOSE_POS = 0.08;
    final double CLAW_MAX_POS = 1; // Maximum rotational position
    final double CLAW_MIN_POS = 0;  // Minimum rotational position

    // arm servo variables, not used in current prototype version.
    private Servo armServo = null;
    final double ARM_FORWARD = 0.2;
    final double ARM_LEFT = 0.4;
    final double ARM_RIGHT = 0.0;

    /**
     * Init slider motors hardware, and set their behaviors.
     * @param hardwareMap the Hardware Mappings.
     * @param armMotorName the name string for arm servo motor
     * @param clawMotorName the name string for claw servo motor
     */
    public void init(HardwareMap hardwareMap, String armMotorName, String clawMotorName) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;

        Logging.log("init servo motors for arm and claw.");
        armServo = hardwareMap.get(Servo.class, armMotorName);
        clawServo = hardwareMap.get(Servo.class, clawMotorName);

        setClawPosition(CLAW_OPEN_POS);

        setArmPosition(ARM_FORWARD);

    }

    /**
     * set the target position of claw servo motor
     * @param clawPos the target position value for claw servo motor
     */
    private void setClawPosition(double clawPos) {
        clawPos = Range.clip(clawPos, CLAW_MIN_POS, CLAW_MAX_POS);
        clawServo.setPosition(clawPos);
    }

    /**
     * set the target position of arm servo motor
     * @param armPos the target position value for arm servo motor
     */
    public void setArmPosition(double armPos) {
        armServo.setPosition(armPos);
    }

    /**
     * set the claw servo motor position to open the claw
     */
    public void clawOpen() {
        setClawPosition(CLAW_OPEN_POS);
    }

    /**
     * set the claw servo motor position to close the claw
     */
    public void clawClose() {
        setClawPosition(CLAW_CLOSE_POS);
    }

    /**
     * Get the arm servo motor current position value
     * @return the current arm servo motor position value
     */
    public double getArmPosition() {
        return armServo.getPosition();
    }

    /**
     * Get the claw servo motor current position value
     * @return the current claw servo motor position value
     */
    public double getClawPosition() {
        return clawServo.getPosition();
    }
}

