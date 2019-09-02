package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 Created 10/3/2018 by C.W

 Updated ?/?/? by C.W- Edited to run teleop

 Updated 11/26/18 by C.W- Remove old arm code and add lifting system

 */
public class HwGM {

    /* Our Public OpMode Devices */
    Servo
            track_servo =null,
            multiEnd_servo =null,
            multiRotate_servo =null;

    DcMotor
            leftMotor = null,
            rightMotor = null,
            left2Motor = null,
            right2Motor = null,
            pullupMotor = null,
            collection_lifter = null;

    final int
            P_UP_DOWN_POS =  0,
            P_UP_RISE_POS =  94419,
            C_UP_DOWN_POS =  0,
            C_UP_RISE_POS = 79112;
    final double
            MEND_ROTATE_SERVOS_STOP = 0.485,
            ROTATE_SERVOS_STOP = 0.485,
            ROTATE_SERVOS_FORWARD_SPEED = 1,
            ROTATE_SERVOS_REVERSE_SPEED = -1,
            TRACK_SERVO_STOP = 0.5,
            TRACK_SERVO_RETRACT_SPEED = 0.9,
            TRACK_SERVO_EXTEND_SPEED = 0.1,
            P_UP_SPEED_FACTOR = 0.75,
            C_UP_SPEED_FACTOR = 0.50;

    /* The OpMode Constants */

    /* The LOCAL OpMode members. */
    HardwareMap hwMap =  null;


    /* Constructor */
    HwGM(){
    }


    /* This Initializes the standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // these are our motors and what they are called
        left2Motor   = hwMap.dcMotor.get("left2_drive");
        right2Motor  = hwMap.dcMotor.get("right2_drive");
        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");


        pullupMotor    = hwMap.dcMotor.get("pullup_motor");
        collection_lifter =hwMap.dcMotor.get("collection_lifter");

        track_servo = hwMap.servo.get("track_servo");
        track_servo.setPosition(0.5);

        multiRotate_servo = hwMap.servo.get("multi_Rotate_servo");
        multiRotate_servo.setPosition(0.485);

        multiEnd_servo = hwMap.servo.get("multi_End_servo");
        multiEnd_servo.setPosition(-0.75);


        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        left2Motor.setDirection(DcMotor.Direction.FORWARD);
        right2Motor.setDirection(DcMotor.Direction.REVERSE);


        pullupMotor.setDirection(DcMotor.Direction.FORWARD);
        collection_lifter.setDirection(DcMotor.Direction.FORWARD);


        leftMotor.setPower(0);
        rightMotor.setPower(0);
        left2Motor.setPower(0);
        right2Motor.setPower(0);

        pullupMotor.setPower(0);
        collection_lifter.setPower(0);
        // we set our drive train motors to run without encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // other motors use encoders
        pullupMotor.setDirection(DcMotor.Direction.REVERSE);
        pullupMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pullupMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        collection_lifter.setDirection(DcMotor.Direction.FORWARD);
        collection_lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collection_lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


}