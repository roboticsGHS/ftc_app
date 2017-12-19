package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@Autonomous(name="armAUTO", group="armAUTO")
public class armAUTO extends LinearOpMode {

    DcMotor leftMotor;    //drive
    DcMotor rightMotor;   //drive
    DcMotor leftLift;
    DcMotor rightLift;
    DcMotor ClawArm;
    Servo servo;  //grabbers
    Servo servo2; //grabbers

    public void moveForward(int time) throws InterruptedException {
        leftMotor.setPower(-0.5);
        rightMotor.setPower(-0.5);
        Thread.sleep(time);
    }

    public void runOpMode() throws InterruptedException {
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");

        //Forward
        leftMotor.setPower(-0.5);
        rightMotor.setPower(-0.5);
        Thread.sleep(2500);
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        Thread.sleep(100);
    }

}

