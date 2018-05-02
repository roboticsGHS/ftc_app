package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="SpringAssembly", group="SpringAssembly")  // @Autonomous(...) is the other common choice
//@Disabled
public class SpringAssembly extends OpMode {


    final static double CLAW_MIN_RANGE  = 0.20;
    final static double CLAW_MAX_RANGE  = 0.6;

    DcMotor leftMotor;    //drive
    DcMotor rightMotor;   //drive
    DcMotor clawArm;

    double blue;
    /**
     * Constructor
     */
    public SpringAssembly() {

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

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        clawArm = hardwareMap.dcMotor.get("clawArm");

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

        float claw_pos = gamepad1.left_trigger;
        float claw_neg = gamepad1.right_trigger;

        // clip the right/left values so that the values never exceed +/- 1
        left_move_value = Range.clip(left_move_value, -1, 1);
        right_move_value = Range.clip(right_move_value, -1, 1);

        claw_pos = (float) Range.clip(claw_pos, -1, 0.5);
        claw_neg = (float) Range.clip(claw_neg, -1, 0);

        //claw_neg = (float) Range.clip(claw_neg, 0,0.5 );
        //right_shooter = Range.clip(right_shooter, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.

        left_move_value =  (float)scaleInput(left_move_value);
        right_move_value = (float)scaleInput(right_move_value);

        claw_pos = (float) scaleInput(claw_pos);
        claw_neg = (float) scaleInput(claw_neg);


        // write the values to the motors
        rightMotor.setPower(right_move_value/2);
        leftMotor.setPower(left_move_value/2);
        clawArm.setPower(claw_pos/1.5);
        clawArm.setPower(claw_neg/1.5);


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