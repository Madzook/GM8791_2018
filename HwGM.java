package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
Created 10/3/2018 by C.W.
 */
public class HwGM {

    /* Our Public OpMode Devices */
    DcMotor
            leftMotor = null,
            rightMotor = null,
            baseArm = null;

    Servo
            armGripper = null,
            midArm = null;

    /* The OpMode Constants */
	final double
            GR_ARM_CLOSED = 0.1,     // All numbers are a GUESS; CHANGE LATER
            GR_GRIPPER_IN = 0.1,
            GR_GRIPPER_OUT = 0.9,
            MID_ARM_UP = 0.9,
            MID_ARM_DOWN =0.1,
            MID_ARM_INIT_POS =  0.2;

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
        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");
        baseArm    = hwMap.dcMotor.get("base_arm");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        baseArm.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        baseArm.setPower(0);


        // we set our drive train motors to run without encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // other motors use encoders
        baseArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // these are our servos and what they are called. The inital servo position is set
        midArm = hwMap.servo.get("middle_arm");
        midArm.setPosition(MID_ARM_INIT_POS);

        armGripper =  hwMap.servo.get("arm_gitter");
        armGripper.setPosition(GR_ARM_CLOSED);
    }


}

