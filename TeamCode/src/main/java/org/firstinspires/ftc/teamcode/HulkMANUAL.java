package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name="HulkMANUAL", group="HulkMANUAL")

public class HulkMANUAL extends OpMode {

    /*
     * Note: the configuration of the servos is such that
     * as the arm grabber approaches 0, the arm position moves up (away from the floor).
     * Also, as the claw grabber approaches 0, the claw opens up (drops the game element).
     */
    // TETRIX VALUES.
    //final static double ARM_MIN_RANGE  = 0.20;
    //final static double ARM_MAX_RANGE  = 0.90;
    //final static double CLAW_MIN_RANGE  = 0.20;
    //final static double CLAW_MAX_RANGE  = 0.6;

    // position of the arm grabber.
    double servoPosition;
    double servoPosition2;
    double speed = .02;
    //double servoPosition2;


    DcMotor leftFront;    //drive
    DcMotor rightFront;   //drive
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor liftArm;
    Servo arm_servo;
    Servo arm_servo2;
    CRServo extend_servo; //grabber arm

    //OpticalDistanceSensor distance;
    //ModernRoboticsI2cGyro gyro;
    //OpticalDistanceSensor lineFollower;
    //TouchSensor touchSensor;
    //ColorSensor colorSensor;
    double blue;

    /**
     * Constructor
     */
    public HulkMANUAL() {

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

      /*
       * For the demo Tetrix K9 bot we assume the following,
       *   There are two motors "motor_1" and "motor_2"
       *   "motor_1" is on the right side of the bot.
       *   "motor_2" is on the left side of the bot.
       *
       * We also assume that there are two servos "servo_1" and "servo_6"
       *    "servo_1" controls the arm joint of the manipulator.
       *    "servo_6" controls the claw joint of the manipulator.
       */

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        liftArm = hardwareMap.dcMotor.get("liftArm");

        arm_servo = hardwareMap.servo.get("armServo");
        arm_servo2 = hardwareMap.servo.get ("armServo2");
        extend_servo = hardwareMap.crservo.get("extendServo");

        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightBack.setDirection(DcMotor.Direction.REVERSE);


        // assign the starting position of the wrist and claw
        servoPosition = 0.5;
        servoPosition2 = 0.5;

    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {


        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left_move_value = gamepad1.left_stick_y;
        float right_move_value = gamepad1.right_stick_y;

        float grabber_lift_pos = gamepad2.right_trigger;
        float grabber_lift_neg = -gamepad2.left_trigger;

        float grabber_extend_value = gamepad2.left_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        left_move_value = Range.clip(left_move_value, -1, 1);
        right_move_value = Range.clip(right_move_value, -1, 1);

        grabber_lift_pos = Range.clip(grabber_lift_pos, -1, 1);
        grabber_lift_neg = Range.clip(grabber_lift_neg, -1, 0);

        left_move_value =  (float)scaleInput(left_move_value);
        right_move_value = (float)scaleInput(right_move_value);

        grabber_lift_pos = (float) scaleInput(grabber_lift_pos);
        grabber_lift_neg = (float) scaleInput(grabber_lift_neg);

        grabber_extend_value = Range.clip(grabber_extend_value,-1, 1);
        grabber_extend_value = (float)scaleInput(grabber_extend_value);

        extend_servo.setPower(grabber_extend_value);


        // write the values to the motors
        rightBack.setPower(right_move_value/2);
        leftBack.setPower(left_move_value/2);

        rightFront.setPower(right_move_value/2);
        leftFront.setPower(left_move_value/2);

        liftArm.setPower(grabber_lift_pos);
        liftArm.setPower (grabber_lift_neg);



        // update the position of the gripper arm
        if (gamepad2.x)
            // if the A button is pushed on gamepad1, increment the position of
            // the gripper grabber.
            servoPosition += speed;
        if (gamepad2.y)
            // if the B button is pushed on5 gamepad1, decrease the position of
            // the gripper grabber.
            servoPosition -= speed;

        if (gamepad2.b)
            // if the A button is pushed on gamepad1, increment the position of
            // the gripper grabber.
        servoPosition2 += speed;
        if (gamepad2.a)
            // if the B button is pushed on5 gamepad1, decrease the position of
            // the gripper grabber.
        servoPosition2 -= speed;

        arm_servo.setPosition(servoPosition);
        servoPosition  = Range.clip(servoPosition, 0, 1);
        arm_servo2.setPosition(servoPosition2);
        servoPosition2  = Range.clip(servoPosition2, 0, 1);


        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left_move_value));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right_move_value));

        //grabber_extend_value
        telemetry.addData("grabber_extend_value",  "grabber_extend_value: " + String.format("%.2f", grabber_extend_value));

    }

    private void scaleInput() {
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

//public class ClawManual extends LinearOpMode {
//}

