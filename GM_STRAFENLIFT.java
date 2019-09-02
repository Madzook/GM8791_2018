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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * Autonomous Opmode GM5
 */

@Autonomous(name="GM_STRAFENLIFT", group="Wired")
//@Disabled
public class GM_STRAFENLIFT extends AutoLinearAbstract {

    // Declare OpMode members specific to this Autonomous Opmode variant.

    @Override
    public void runOpMode() {

        // Execute the typical autonomous program elements.
        // super.runOpMode finishes as soon as the Drive Station start/play button is pressed.
        super.runOpMode();

        //lowers lifts




        pullup_motor.goToAbsoluteDistance(5, .5);
        step = 1;
        while (!pullup_motor.isMoveDone(LANDING_ERROR)) {
            telemetry.addLine("wait-landing");
            motorTelemetry(pullup_motor);
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                pullup_motor.stop();
                break;
            }
        }



        // Go straight to exit balancing stone
        driveTrain.goStraightToTarget(2, DRIVE_TRAIN_DEFAULT_SPEED);
        step = 2;
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move straight to exit balancing stone");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }


        pullup_motor.goToAbsoluteDistance(0, .5);



        driveTrain.resetEncoders();

        driveTrain.goStraightToTarget(19, DRIVE_TRAIN_DEFAULT_SPEED);
        step = 3;
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive straight to elements");
            colorSensor1.evaluateColor();
            colorSensor2.evaluateColor();
            colorSensor3.evaluateColor();
            colorSensor4.evaluateColor();

            ColorTelemetry();
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }


        if (colorSensor1.colorGold || colorSensor2.colorGold || colorSensor3.colorGold || colorSensor4.colorGold) {
            driveTrain.goStraightToTarget(25, .5);
            step = 4;
        }
        else {
            driveTrain.StrafeLeftToTarget(18, .5);
            step = 5;
        }
        while(!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES) ) {
            telemetry.addLine("Wait - step 4/5");
            ColorTelemetry();
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        colorSensor1.evaluateColor();
        colorSensor2.evaluateColor();
        colorSensor3.evaluateColor();
        colorSensor4.evaluateColor();

        if (step ==5 && (colorSensor1.colorGold || colorSensor2.colorGold || colorSensor3.colorGold || colorSensor4.colorGold)) {
            driveTrain.goRoundToTarget(60, 37, 0.5);
            step = 6;
        }

        else {
            if (step == 5) {
                driveTrain.StrafeRightToTarget(36, .5);
                step = 7;
            }
        }
        while(!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES) ) {
            telemetry.addLine("Wait - Drive straight to elements");
            ColorTelemetry();
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        if (step ==7) {
            driveTrain.goRoundToTarget(60, 37, 0.5);
            step = 8;
        }

        while(!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES) ) {
            telemetry.addLine("Wait - Drive straight to elements");
            ColorTelemetry();
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }


        multi_End_servo.goToPosition(.8, 0.02);
        generalTimer.reset();
        multi_Rotate_servo.goToPositionNow(.9);
        while (generalTimer.seconds() < 5) {
            telemetry.addLine("Servo dumps element ");

            if (autoTimer.seconds() > 28) {
                break;
            }
        }

        multi_Rotate_servo.goToPositionNow(.485);


        while (!pullup_motor.isMoveDone(LANDING_ERROR)) {
            telemetry.addLine("reset-lander");
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }


    }
}

