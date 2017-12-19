package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name="TriangleMANUAL", group="TriangleMANUAL")

public class TriangleMANUAL extends OpMode {

    /*
     * Note: the configuration of the servos is such that
     * as the arm servo approaches 0, the arm position moves up (away from the floor).
     * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
     */
    // TETRIX VALUES.
    //final static double ARM_MIN_RANGE  = 0.20;
    //final static double ARM_MAX_RANGE  = 0.90;
    final static double CLAW_MIN_RANGE  = 0.20;
    final static double CLAW_MAX_RANGE  = 0.6;

    // position of the arm servo.
    double servoPosition;
    double speed = .02;
    double servoPosition2;
    double fallPosition;
    double fallPosition2;



    DcMotor leftMotor;    //drive
    DcMotor rightMotor;   //drive

    DcMotor leftLift;
    DcMotor rightLift;

    Servo servo;  //grabbers
    Servo servo2; //grabbers

    //OpticalDistanceSensor distance;
    //ModernRoboticsI2cGyro gyro;
    //OpticalDistanceSensor lineFollower;
    //TouchSensor touchSensor;
    //ColorSensor colorSensor;
    double blue;
    /**
     * Constructor
     */
    public TriangleMANUAL() {

    }


    @Override
    public void init() {

        //rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        //leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        //leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //ClawArm1 = hardwareMap.dcMotor.get("ClawArm1");
        //ClawArm2 = hardwareMap.dcMotor.get("ClawArm2");
        //rightShooter.setDirection(DcMotor.Direction.REVERSE);

        leftLift = hardwareMap.dcMotor.get("leftLift");
        rightLift = hardwareMap.dcMotor.get("rightLift");
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        //lineFollower = hardwareMap.opticalDistanceSensor.get("lineFollower");
        //distance = hardwareMap.opticalDistanceSensor.get("distance");
        //colorSensor = hardwareMap.colorSensor.get("colorSensor");
        //gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        servo2 = hardwareMap.servo.get("servo2");
        servo2.setPosition(servoPosition2);

        servo = hardwareMap.servo.get("servo");
        servo.setPosition(servoPosition);

       /* fall = hardwareMap.servo.get("fall");
        fall.setPosition(fallPosition);

        fall2 = hardwareMap.servo.get("fall2");
        fall2.setPosition(fallPosition2);
*/
        // assign the starting position of the wrist and claw
        servoPosition = .4;
        servoPosition2 = .4;
        //fallPosition = .4;
        //fallPosition2 = .4;

    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {


        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;

        float lift_pos = gamepad2.left_stick_y;
        float lift_neg = -gamepad2.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);

        lift_pos = Range.clip(lift_pos, -1, 1);
        lift_neg = Range.clip(lift_neg, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        left =  (float)scaleInput(left);
        right = (float)scaleInput(right);


        //left_2 =  (float)scaleInput(left_2);
        //right_2 = (float)scaleInput(right_2);

        lift_pos = (float) scaleInput(lift_pos);
        lift_neg = (float) scaleInput(lift_neg);


        // write the values to the motors
        //rightFrontMotor.setPower(right);
        //leftFrontMotor.setPower(left);
        rightMotor.setPower(right/2);
        leftMotor.setPower(left/2);

        leftLift.setPower(lift_pos);
        rightLift.setPower(lift_pos);
        leftLift.setPower(lift_neg);
        rightLift.setPower(lift_neg);


        //if (gamepad1.right_bumper)
        // {
        // if the A button is pushed on gamepad1, increment the position of
        // the arm servo.
        //    ClawArm.setPower(1);
        // }
        //else if (!gamepad1.right_bumper)
        // {
        //    ClawArm.setPower(0);
//
//    }

//    if (gamepad1.left_bumper)
//    {
        // if the Y button is pushed on5 gamepad1, decrease the position of
        // the arm servo.
//       ClawArm.setPower(-1);
//    }
//    else if (!gamepad1.left_bumper)
//    {
//       ClawArm.setPower(0);
//    }

        // update the position of the arm.
        if (gamepad2.a)
            // if the LB button is pushed on gamepad2, increment the position of
            // the gripper servo.
            servoPosition += speed;

        if (gamepad2.x)
            // if the LB button is pushed on5 gamepad2, decrease the position of
            // the gripper servo.
            servoPosition += speed;
        servo.setPosition(servoPosition/2);
        servoPosition  = Range.clip(servoPosition, 0, 1);

        servo2.setPosition(servoPosition2/2);
        servoPosition2  = Range.clip(servoPosition2, 0, 1);

        if (gamepad2.b)
            // if the X button is pushed on gamepad1, increment the position of
            // the griper servo.
            servoPosition2 += speed;

        else if (gamepad2.y)
            // if the Y button is pushed on5 gamepad1, decrease the position of
            // the gripper servo.
            servoPosition2 -= speed;

        // Fall servos will be activated when
        /*if (gamepad2.x)
            // if the LB button is pushed on gamepad2, increment the position of
            // the gripper servo.
            fallPosition -= speed;
        if (gamepad2.y)
            // if the LB button is pushed on5 gamepad2, decrease the position of
            // the gripper servo.
            fallPosition2 += speed;
        fall.setPosition(fallPosition/2);
        fallPosition  = Range.clip(fallPosition, 0, 1);

        fall2.setPosition(fallPosition2/2);
        fallPosition2  = Range.clip(fallPosition2, 0, 1);
*/


//


      /*
       * Send telemetry data back to driver station. Note that if we are using
       * a legacy NXT-compatible motor controller, then the getPower() method
       * will return a null value. The legacy NXT-compatible motor controllers
       * are currently write only.
       */

      /*if ( button.isPressed() ) {



        }*/

        telemetry.addData("Text", "*** Robot Data***");
        //telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
        //telemetry.addData("claw", "claw:  " + String.format("%.2f", ClawArm));
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));

    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}

/**
 * Created by Team 10123 on 11/13/2017.
 */

//public class TrianlgeManual extends LinearOpMode {
//}

