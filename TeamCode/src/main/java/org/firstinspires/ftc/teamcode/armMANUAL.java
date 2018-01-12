package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="armMANUAL", group="armMANUAL")  // @Autonomous(...) is the other common choice
//@Disabled
public class armMANUAL extends OpMode {


    final static double CLAW_MIN_RANGE  = 0.20;
    final static double CLAW_MAX_RANGE  = 0.6;

    // position of the arm servo.
    double servoPosition;
    double speed = .02;
    double servoPosition2;



    DcMotor leftMotor;    //drive
    DcMotor rightMotor;   //drive
    DcMotor leftFront;    //frontdrive
    DcMotor rightFront;   //frontdrive
    DcMotor leftLift;
    DcMotor rightLift;
    DcMotor ClawArm;
    Servo servo;  //grabbers
    Servo servo2; //grabbers
    //Servo claw; //lift arm
    //OpticalDistanceSensor distance;
    //ModernRoboticsI2cGyro gyro;
    //OpticalDistanceSensor lineFollower;
    //TouchSensor touchSensor;
    //ColorSensor colorSensor;
    double blue;
    /**
     * Constructor
     */
    public armMANUAL() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
      /*
       * Use the hardwareMap to get the dc motors and servos by name. Note
       * that the names of the devices must match the names used when you
       * configured your robot and created the configuration file.
       */

        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        ClawArm = hardwareMap.dcMotor.get("ClawArm");
        //rightShooter = hardwareMap.dcMotor.get("rightShooter");
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

        // assign the starting position of the wrist and claw
        servoPosition = 0.5;
        servoPosition2 = .4;
        //clawPosition = 0.1;

    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

      /*
       Gamepad 1
        */
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left_move_value = gamepad1.left_stick_y;
        float right_move_value = gamepad1.right_stick_y;

        float claw_pos = gamepad2.left_stick_y;

        float lift_pos = gamepad1.right_trigger;
        float lift_neg = -gamepad1.left_trigger;

        // clip the right/left values so that the values never exceed +/- 1
        left_move_value = Range.clip(left_move_value, -1, 1);
        right_move_value = Range.clip(right_move_value, -1, 1);

        lift_pos = Range.clip(lift_pos, -1, 1);
        lift_neg = Range.clip(lift_neg, -1, 0);

        claw_pos = (float) Range.clip(claw_pos, -1, 0.5);
        //claw_neg = (float) Range.clip(claw_neg, -1, 0);

        //claw_neg = (float) Range.clip(claw_neg, 0,0.5 );
        //right_shooter = Range.clip(right_shooter, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.

        left_move_value =  (float)scaleInput(left_move_value);
        right_move_value = (float)scaleInput(right_move_value);

        //left_2 =  (float)scaleInput(left_2);
        //right_2 = (float)scaleInput(right_2);

        lift_pos = (float) scaleInput(lift_pos);
        lift_neg = (float) scaleInput(lift_neg);

        claw_pos = (float) scaleInput(claw_pos);
        //claw_neg = (float) scaleInput(claw_neg);


        // write the values to the motors
        rightFront.setPower(right_move_value/2);
        leftFront.setPower(left_move_value/2);
        rightMotor.setPower(right_move_value/2);
        leftMotor.setPower(left_move_value/2);

        ClawArm.setPower(claw_pos);
        //ClawArm.setPower(claw_neg);

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
        if (gamepad1.a)
            // if the A button is pushed on gamepad1, increment the position of
            // the gripper servo.
            servoPosition += speed;

        if (gamepad1.y)
            // if the B button is pushed on5 gamepad1, decrease the position of
            // the gripper servo.
            servoPosition -= speed;


        servo.setPosition(servoPosition);
        servoPosition  = Range.clip(servoPosition, 0, 1);

        servo2.setPosition(servoPosition2);
        servoPosition2  = Range.clip(servoPosition2, 0, 1);
        if (gamepad1.b)
            // if the X button is pushed on gamepad1, increment the position of
            // the griper servo.
            servoPosition2 += speed;

        else if (gamepad1.x)
            // if the Y button is pushed on5 gamepad1, decrease the position of
            // the gripper servo.
            servoPosition2 -= speed;

//
        // clip the position values so that they never exceed their allowed range.
        //armPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
        //ClawArm = Range.clip(CLAW_MIN_RANGE, CLAW_MAX_RANGE);

        // write position values to the wrist and claw servo
        //arm.setPosition(armPosition);
        //claw.setPosition(clawPosition);

      /*
       * Send telemetry data back to driver station. Note that if we are using
       * a legacy NXT-compatible motor controller, then the getPower() method
       * will return a null value. The legacy NXT-compatible motor controllers
       * are currently write only.
       */

      /*if ( button.isPressed() ) {



        }*/

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left_move_value));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right_move_value));

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
 * Created by Team 8856 on 11/13/2017.
 */

//public class ArmManual extends LinearOpMode {
//}