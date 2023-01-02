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


/**
 * This is NOT an opmode.
 *
 * This class defines the parameters related with game field elements.
 */
public class Params
{
    // slider position variables
    static final double ARM_UNLOADING_LIFTING = 4; // Arm lifting from loading position to unloading position
    static final double GROUND_CONE_POSITION = 0.0;
    static final double coneLoadStackGap = 1.3;
    static final double coneStack5th = coneLoadStackGap * 4;
    static final double GROUND_JUNCTION_POS = GROUND_CONE_POSITION + 1.0;
    static final double LOW_JUNCTION_POS = 13.5  - ARM_UNLOADING_LIFTING;
    static final double MEDIUM_JUNCTION_POS = 23.5  - ARM_UNLOADING_LIFTING;
    static final double HIGH_JUNCTION_POS = 33.5 - ARM_UNLOADING_LIFTING;
    static final double WALL_POSITION = 7.5 ;
    static final double SLIDER_MOVE_DOWN_POSITION = 4.0;

    // autonomous driving distance
    static final double HALF_MAT = 12.0;
    static final double BASE_TO_JUNCTION = 2 * HALF_MAT - 2.0;
    static final double CHASSIS_WIDTH = 14;
    static final double CHASSIS_LENGTH = 16;
    static final double ARM_LENGTH = 12;
    static final double ARM_LOCATION_BIAS = 2.0; // Arm joint location to the center of chassis.
    static final double V_DISTANCE_TO_CENTER = 6.5; // distance from V to the center of the robot
    static final double INIT_POSITION_TO_MAT_CENTER = 5 * HALF_MAT - CHASSIS_WIDTH / 2.0;
    static final double INIT_POSITION_TO_2ND_MAT_EDGE = 4 * HALF_MAT - CHASSIS_WIDTH / 2.0;
    static final double HIGH_JUNCTION_TO_CONE_STACK = 4 * HALF_MAT - ARM_LENGTH;
}

