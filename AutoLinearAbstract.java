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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Abstract Linear Autonomous OpMode
 * Created 19Nov2017 for the Greenwood FTC Robotics Club.
 * This class provides relicRobot status and methods typical to all Relic Game autonomous OpModes
 *
 *
 * Revised 18Dec2017 - Center Grove competition adjustments
 * Revised 23Jan2018 - Add JewelArm2 servo
 * Revised 13Feb2018 - Modified Glyph Arm gear ratio and speed
 * Revised 15Feb2018 - Increased servo speed increments
 *                   - Added servoRelicLinear and initialization logic*/


public abstract class AutoLinearAbstract extends LinearOpMode {

    /* =======================================================
     * CLASS MEMBERS (i.e., Class Status)
     * Common autonomous opmode members
     * ======================================================= */

    /* -------------------------------------------------------
     * Public (Shared) Class Members
     * Automated objects, timers, variables, constants
     * ------------------------------------------------------- */

    // OBJECTS
   mecanumDrive
            driveTrain;


    DeviceTargetMotor
            pullup_motor,
            collection_lifter;

     DeviceColorSensor
           colorSensor1,
           colorSensor2,
            colorSensor3,
            colorSensor4;

            DeviceTargetServo
            track_servo,
            multi_Rotate_servo,
            multi_End_servo;

    ElapsedTime
            blinkTimer = new ElapsedTime(),   // BlinkTimer
            generalTimer = new ElapsedTime(), // General/multipurpose timer
            autoTimer = new ElapsedTime();    // Autonomous timer

    int
            step,
            colordetected;



    // CONSTANTS
    static final int
            G_ARM_TRAVEL_DEGREES = 20;

    final static double
            MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES = 0.25,
            GLYPH_ARM_TRAVEL_ANGLE = 30,
            GLYPH_ARM_HIGH_TRAVEL_ANGLE = 70,
            LANDING_ERROR = 1.0,

            DRIVE_TRAIN_PIVOT_SPEED = 0.2,
            DRIVE_TRAIN_DEFAULT_SPEED = 0.5,
            DRIVE_TRAIN_STRAIGHT_SPEED = 0.6,
            GLYPH_ARM_DEFAULT_SPEED = 0.3,


            CONTINUOUS_SERVO_INIT_SPEED=0.5,
            OBJECT_SERVO_INIT_POS = 0.05,
            J_ARM_UP_POS =  0.7,
            J_ARM_DOWN_POS =  0,
            J_ARM_SWIPE_POS =  0.02,
            J_ARM_INTERMEDIATE_POS =  0.4,
            J_ARM_INC_PER_SCAN = 0.01,
            J_ARM_INC_PER_SCAN_FAST = 0.03,

            J_ARM2_RIGHT = 0.8,
            J_ARM2_LEFT = 0.2,
            J_ARM2_RETURN = 0.5,
            J_ARM2_INC_PER_SCAN = 0.01,

            G_GRIPPER_OPEN = 0.6,
            G_GRIPPER_CLOSED = 1.0,
            G_GRIPPER_INC_PER_SCAN = 0.02,

            R_GRABBER_CLOSED = 0.85,
            R_LINEAR_IN_POS = 0.1,
            R_LINEAR_OUT_POS = 0.9,

            BLINK_OFF_PERIOD = 0.3,
            BLINK_ON_PERIOD = 0.3;


    final static boolean
            FORWARD = false,
            REVERSE = true,
            LIGHT_ON = true,
            LIGHT_OFF = false;


    /* =======================================================
     * CLASS METHODS (i.e., Class Behavior)
     * ======================================================= */

    /* -------------------------------------------------------
     * Method: runOpMode (Overridden Linear OpMode)
     * Purpose: Establish and initialize automated objects, and signal initialization completion
     * ------------------------------------------------------- */
    @Override
    public void runOpMode() {

        /* INITIALIZE ROBOT - ESTABLISH ROBOT OBJECTS */

        // Notify drive station that robot objects are being built
        telemetry.addLine("Wait - Building Robot Objects");
        telemetry.update();

        /* Drive Train constructor: hardwareMap, left motor name, left motor direction, right motor name, right motor direction,
                                    encoder counts per output shaft revolution, gear ratio, wheel radius */
        driveTrain = new mecanumDrive(hardwareMap,"left_drive",REVERSE,"right_drive",FORWARD,"left2_drive",REVERSE,"right2_drive",FORWARD,1440,1.0,2.0);

        /* Target-Motor constructor: hardwareMap, motor name, motor direction,
                              encoder counts per output shaft revolution, gear ratio, wheel radius */
        pullup_motor = new DeviceTargetMotor(hardwareMap,"pullup_motor",REVERSE,560,1.5,0.0769);


        /* Color sensor constructor: hardwareMap, sensor name, sensor I2C address */
        colorSensor1 = new DeviceColorSensor(hardwareMap,"colorSensor1",0x39);
        colorSensor2 = new DeviceColorSensor(hardwareMap,"colorSensor2",0x39);
        colorSensor3 = new DeviceColorSensor(hardwareMap,"colorSensor3",0x39);
        colorSensor4 = new DeviceColorSensor(hardwareMap,"colorSensor4",0x39);
        /* Target-Servo constructor: hardwareMap, servo name, initial servo position */
        track_servo = new DeviceTargetServo(hardwareMap,"track_servo", CONTINUOUS_SERVO_INIT_SPEED);
        multi_Rotate_servo = new DeviceTargetServo(hardwareMap,"multi_Rotate_servo",0.485);
        multi_End_servo = new DeviceTargetServo(hardwareMap,"multi_End_servo",OBJECT_SERVO_INIT_POS);
         /* INITIALIZE ROBOT - INITIALIZE ROBOT OBJECTS AND CLASSES*/

        // Notify drive station that robot objects are being initialized
        telemetry.addLine("Wait - Initializing Robot Objects");
        telemetry.update();

         /* Reset encoders and place motors into the 'Run-to-Position' mode
            Note: The initialization calls in the following methods could not be performed in the respective
           object constructors */
        driveTrain.resetEncoders();
        pullup_motor.resetEncoder();


        /* Complete color sensor setup
           Note: These initialization methods could not be performed in the respective object constructors */


        // Note: Servo initialization is completed in the respective object constructors
       step = 0;
       colordetected = 0;

        /* INITIALIZE ROBOT - SIGNAL INITIALIZATION COMPLETE */

        // Notify drive station that initialization is wrapping up
        telemetry.addLine("Wait - Initializing Almost Complete");
        telemetry.update();


        for (int index = 0; index < 2; index++){
            // blink off lights
            blinkTimer.reset();
            colorSensor1.turnOnLight();
            colorSensor1.turnOffLight();

            while (blinkTimer.seconds() < BLINK_OFF_PERIOD) {}

            // blink on lights
            blinkTimer.reset();
            colorSensor1.turnOnLight();
            colorSensor2.turnOnLight();
            while (blinkTimer.seconds() < BLINK_ON_PERIOD) {}



            // Note: Lights/LEDs will remain on after the FOR loop completes
        }



        // Report initialization complete
        telemetry.addLine("Initialization Complete");
        telemetry.addLine("Hold for Start");
        telemetry.update();


        // WAIT FOR THE GAME TO START (driver presses PLAY)
        waitForStart();

        autoTimer.reset();  // Reset/restart the autotimer



        // GAME STARTED - BEGIN AUTONOMOUS OPERATIONS

    }



    /* -------------------------------------------------------
     * Method: driveTrainTelemetry
     * Purpose: Report the position and speed of the drive train wheels
     * ------------------------------------------------------- */
    void driveTrainTelemetry () {
        telemetry.addLine();
        telemetry.addLine("Left Drive Motor");
        telemetry.addData("  Position in EngUnits","%.2f", driveTrain.front.motorLeft.getPosition());
        telemetry.addData("  Target in EngUnits","%.2f", driveTrain.front.motorLeft.targetPosition);
        telemetry.addData("  Position in Counts",driveTrain.front.motorLeft.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts",driveTrain.front.motorLeft.targetCount);
        telemetry.addData("  Is Busy",driveTrain.front.motorLeft.targetMotor.isBusy());
        telemetry.addData("  Speed",driveTrain.front.leftSpeed);
        telemetry.addLine();
        telemetry.addLine("Right Drive Motor");
        telemetry.addData("  Position in EngUnits","%.2f", driveTrain.front.motorRight.getPosition());
        telemetry.addData("  Target in EngUnits","%.2f", driveTrain.front.motorRight.targetPosition);
        telemetry.addData("  Position in Counts",driveTrain.front.motorRight.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts",driveTrain.front.motorRight.targetCount);
        telemetry.addData("  Is Busy",driveTrain.front.motorRight.targetMotor.isBusy());
        telemetry.addData("  Speed",driveTrain.front.rightSpeed);





        telemetry.addLine();
        telemetry.addLine("Left Drive Motor");
        telemetry.addData("  Position in EngUnits","%.2f", driveTrain.rear.motorLeft.getPosition());
        telemetry.addData("  Target in EngUnits","%.2f", driveTrain.rear.motorLeft.targetPosition);
        telemetry.addData("  Position in Counts",driveTrain.rear.motorLeft.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts",driveTrain.rear.motorLeft.targetCount);
        telemetry.addData("  Is Busy",driveTrain.rear.motorLeft.targetMotor.isBusy());
        telemetry.addData("  Speed",driveTrain.rear.leftSpeed);
        telemetry.addLine();
        telemetry.addLine("Right Drive Motor");
        telemetry.addData("  Position in EngUnits","%.2f", driveTrain.rear.motorRight.getPosition());
        telemetry.addData("  Target in EngUnits","%.2f", driveTrain.rear.motorRight.targetPosition);
        telemetry.addData("  Position in Counts",driveTrain.rear.motorRight.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts",driveTrain.rear.motorRight.targetCount);
        telemetry.addData("  Is Busy",driveTrain.rear.motorRight.targetMotor.isBusy());
        telemetry.addData("  Speed",driveTrain.rear.rightSpeed);
    }


    void motorTelemetry (DeviceTargetMotor motor) {
        telemetry.addLine();
        telemetry.addData("Motor Name: ",motor.name);
        telemetry.addData("  Position in EngUnits","%.2f", motor.getPosition());
        telemetry.addData("  Target in EngUnits","%.2f", motor.targetPosition);
        telemetry.addData("  Position in Counts",motor.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts",motor.targetCount);
        telemetry.addData("  Is Busy",motor.targetMotor.isBusy());
        telemetry.addData("  Speed",motor.targetMotor.getPower());
        telemetry.addLine();

    }






    void motorTelemetryDegrees (DeviceTargetMotor motor) {
        telemetry.addLine();
        telemetry.addLine(motor.name);
        telemetry.addData(" Position in Degrees", "%.2f degrees ", motor.getDegrees());
        telemetry.addData(" Position in Counts", motor.targetMotor.getCurrentPosition());
    }


   void ColorTelemetry () {
        telemetry.addLine();
        telemetry.addLine("Color sensor 1");
        telemetry.addData("  Hue", "%.2f", colorSensor1.hue);
        telemetry.addData("  Saturation", "%.4f", colorSensor1.saturation);
        telemetry.addData("  Value", "%.4f", colorSensor1.value);
        telemetry.addData("  White", colorSensor1.colorWhite);
        telemetry.addData("  Gold", colorSensor1.colorGold);
        telemetry.addLine();
        telemetry.addLine("Color sensor 2");
        telemetry.addData("  Hue", "%.2f", colorSensor2.hue);
        telemetry.addData("  Saturation", "%.4f", colorSensor2.saturation);
        telemetry.addData("  Value", "%.4f", colorSensor2.value);
        telemetry.addData("  White", colorSensor2.colorWhite);
        telemetry.addData("  Gold", colorSensor2.colorGold);
    }

}
