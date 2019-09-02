package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="GM TeleOpMecanum", group="GreenwoodFTC")  //declares the name of the class and the
// group it is in.
//@Disabled


public class GMTeleOpMecanum extends OpMode{


    /*declaring our robot as an object*/
    HwGM robot = new HwGM();


    double  //declares all double variables and their values
            trackServoPosition = robot.TRACK_SERVO_EXTEND_SPEED,
            left,
            right,
            MEND_POSITION = 0.5,
            pullupspeed = 0,
            collectionspeed = 0;


    private boolean //declares all private booleans and their initial values (true or false)
            brakeCollection = true,
            brakePullUp = true;



    /*
     * Code will run ONCE when the driver hits INIT
     * INIT means initialize
     */


    @Override

    public void init() { //initialization class to be used at start of tele-op

        robot.init(hardwareMap); // This will initialize the robot object via it's init method

        //this will send a telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //

    }

    /*
     * Code will run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */


    @Override

    public void init_loop() {
    }

    /*
     *this code will run ONCE when the driver hits PLAY
     */

    @Override

    public void start() {
    }

    /*
     * Code will run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */


    @Override

    public void loop() {

//==========================================================
//						GamePad One
//==========================================================

        float LFspeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
        float LBspeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
        float RFspeed = gamepad1.right_stick_y + gamepad1.right_stick_x;
        float RBspeed = gamepad1.right_stick_y - gamepad1.right_stick_x;


        LFspeed = Range.clip(LFspeed, -1, 1);
        LBspeed = Range.clip(LBspeed, -1, 1);
        RFspeed = Range.clip(RFspeed, -1, 1);
        RBspeed = Range.clip(RBspeed, -1, 1);



        robot.left2Motor.setPower(LFspeed);
        robot.right2Motor.setPower(RFspeed);
        robot.leftMotor.setPower(LBspeed);
        robot.rightMotor.setPower(RBspeed);

//Controls the pull-up system of robot

        if (gamepad1.right_trigger >= 0.10) {  //if the right trigger value is greater than .1, the
            // pullup arm will go upwards, allowing for re-
            robot.pullupMotor.setTargetPosition(robot.P_UP_DOWN_POS); // attatchment to the lander
            pullupspeed = gamepad1.right_trigger * robot.P_UP_SPEED_FACTOR;
            robot.pullupMotor.setPower(pullupspeed);}

        if (gamepad1.left_trigger >= 0.10) {

            robot.pullupMotor.setTargetPosition(robot.P_UP_RISE_POS);
            pullupspeed = gamepad1.left_trigger * robot.P_UP_SPEED_FACTOR;
            robot.pullupMotor.setPower(pullupspeed);}

        if (gamepad1.left_trigger < 0.10 && gamepad1.right_trigger < 0.10) {

            if (!brakePullUp) {

                brakePullUp = true;
                robot.pullupMotor.setTargetPosition(robot.pullupMotor.getCurrentPosition());
                robot.pullupMotor.setPower(robot.P_UP_SPEED_FACTOR);}
        }

        else

            brakePullUp = false;



//==========================================================
//						GamePad Two
//==========================================================
//Change Collection Lift to joysticks/ add new gear ratio 4.5:1 with new motor never rest 60


        //Control Lienar extender
        if (gamepad2.left_bumper){
            telemetry.addLine("Servo Track Extending");
            robot.track_servo.setPosition(robot.TRACK_SERVO_EXTEND_SPEED);}

        if (gamepad2.right_bumper) {
            robot.track_servo.setPosition(robot.TRACK_SERVO_RETRACT_SPEED);
            telemetry.addLine("Servo Track Retracting");}

        if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
            telemetry.addLine("Servo Track Stop");
            robot.track_servo.setPosition(robot.TRACK_SERVO_STOP);}

        //Lifts Lienar extender

        if (gamepad2.right_trigger >= 0.10) {

            robot.collection_lifter.setTargetPosition(robot.C_UP_DOWN_POS);
            collectionspeed = gamepad2.right_trigger * robot.C_UP_SPEED_FACTOR;
            robot.collection_lifter.setPower(collectionspeed);}


        if (gamepad2.left_trigger >= 0.10) {

            robot.collection_lifter.setTargetPosition(robot.C_UP_RISE_POS);
            collectionspeed = gamepad2.left_trigger * robot.C_UP_SPEED_FACTOR;
            robot.collection_lifter.setPower(collectionspeed);}

        if (gamepad2.left_trigger < 0.10 && gamepad2.right_trigger < 0.10) {

            if (!brakeCollection) {

                brakeCollection = true;
                robot.collection_lifter.setTargetPosition(robot.collection_lifter.getCurrentPosition());
                robot.collection_lifter.setPower(robot.C_UP_SPEED_FACTOR);}
        }
        else

            brakeCollection = false;

        //Multi Rotation system

        if (gamepad2.b){

            robot.multiRotate_servo.setPosition(robot.ROTATE_SERVOS_FORWARD_SPEED);}


        if (gamepad2.x){

            robot.multiRotate_servo.setPosition(robot.ROTATE_SERVOS_REVERSE_SPEED);}


        if (!gamepad2.b && !gamepad2.x){

            robot.multiRotate_servo.setPosition(robot.ROTATE_SERVOS_STOP);}



        //Multi End rotation system

        if (gamepad2.dpad_right) {
            if (MEND_POSITION > 0.95)
                robot.multiEnd_servo.setPosition(MEND_POSITION = MEND_POSITION + 0.01);
            else
                robot.multiEnd_servo.setPosition(MEND_POSITION);
        }

        if (gamepad2.dpad_left){
            if (MEND_POSITION < 0.25)
                robot.multiEnd_servo.setPosition(MEND_POSITION = MEND_POSITION - 0.01);
            else
                robot.multiEnd_servo.setPosition(MEND_POSITION);
        }





        //==========================================================
        //						Telemetry
        //==========================================================

        // this telemetry will report power setpoint to the Driver Station phone

        telemetry.addData("left",  "%.2f", left);

        telemetry.addData("right", "%.2f", right);


        telemetry.addData("Left Trigger", "%.2f", gamepad1.left_trigger);

        telemetry.addData("Right Trigger", "%.2f", gamepad1.right_trigger);

        telemetry.addData("CollectionLift Speed", "%.2f", collectionspeed);

        telemetry.addData("CollectionLift Pos",  robot.collection_lifter.getCurrentPosition());

        telemetry.addData("Pull Up Speed", "%.2f", pullupspeed);

        telemetry.addData("PullUpPos",  robot.pullupMotor.getCurrentPosition());

        telemetry.addData("Left Bumper", gamepad1.left_bumper);

        telemetry.addData("Right Bumper", gamepad1.right_bumper);

        telemetry.addData("RFMotor", robot.right2Motor.getCurrentPosition());
        telemetry.addData("RBMotor", robot.rightMotor.getCurrentPosition());

        telemetry.addData("LFMotor", robot.left2Motor.getCurrentPosition());
        telemetry.addData("LBMotor", robot.leftMotor.getCurrentPosition());
    }

    /*
     * Code will run ONCE after the driver hits STOP
     */

    @Override

    public void stop(){


        // Sets all motors to zero power

        robot.leftMotor.setPower(0);
        robot.left2Motor.setPower(0);

        robot.rightMotor.setPower(0);
        robot.right2Motor.setPower(0);


    }

} //end main