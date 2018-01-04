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
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        Thread.sleep(time);
    }

    public void moveRight(int time) throws InterruptedException {
        leftLift.setPower(0.5);
        rightLift.setPower(0.5);
        Thread.sleep(time);
    }
    public void moveBackwards(int time) throws InterruptedException {
        leftMotor.setPower(-0.5);
        rightMotor.setPower(-0.5);
        Thread.sleep(time);
    }
    public void turnLeft (int time) throws InterruptedException {
        rightMotor.setPower(0.5);
        Thread.sleep(time);
    }

    public void turnRight(int time) throws InterruptedException {
        leftMotor.setPower(0.5);
        Thread.sleep(time);
    }

    public void releaseServo(int time) throws InterruptedException {
        servo.setPosition(0.4);
        Thread.sleep(time);
    }

    public void runOpMode() throws InterruptedException {
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        leftLift = hardwareMap.dcMotor.get ("leftLift");
        rightLift = hardwareMap.dcMotor.get ("rightLift");
        servo = hardwareMap.servo.get ("servo");
        servo2 = hardwareMap.servo.get ("servo2");

        //Backwards
        moveBackwards(2500);



    }

}

