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

    DcMotor leftMotor;    //backdrive
    DcMotor rightMotor;   //backdrive
    DcMotor leftFront;    //frontdrive
    DcMotor rightFront;   //frontdrive
    DcMotor leftLift;
    DcMotor rightLift;
    DcMotor ClawArm;
    Servo servo;  //grabbers
    Servo servo2; //grabbers

    public void moveForward(int time) throws InterruptedException {
        leftMotor.setPower(-0.5);
        rightMotor.setPower(0.5);
        leftFront.setPower(0.5);
        rightFront.setPower(-0.5);
        Thread.sleep(time);
    }

    public void moveRight(int time) throws InterruptedException {
        leftLift.setPower(0.5);
        rightLift.setPower(-0.5);
        Thread.sleep(time);
    }
    public void moveLeft (int time) throws InterruptedException {
        leftLift.setPower(-0.5);
        rightLift.setPower(0.5);
        Thread.sleep(time);
    }
    public void moveBackwards(int time) throws InterruptedException {
        leftMotor.setPower(0.5);
        rightMotor.setPower(-0.5);
        leftFront.setPower(-0.5);
        rightFront.setPower(0.5);
        Thread.sleep(time);
    }
    public void turnRight (int time) throws InterruptedException {
        rightMotor.setPower(-0.5);
        rightFront.setPower(0.5);
        leftMotor.setPower(-0.5);
        leftFront.setPower(0.5);
        Thread.sleep(time);
    }

    public void turnLeft(int time) throws InterruptedException {
        rightMotor.setPower(0.5);
        rightFront.setPower(-0.5);
        leftMotor.setPower(0.5);
        leftFront.setPower(-0.5);
        Thread.sleep(time);
    }

    public void OpenServo(double position) throws InterruptedException {
        servo.setPosition(position);
        servo2.setPosition(position);
    }
    public void closeServo() {
        servo.setPosition(0.9); //left servo needs to be higher number to close
        servo2.setPosition(0.1); //right servo needs to be a low number to close
    }
    public void raiseArm() {
        ClawArm.setPower(0.5);
    }
    public void lowerArm() {
        ClawArm.setPower(-0.5);
    }
    public void runOpMode() throws InterruptedException {
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftLift = hardwareMap.dcMotor.get ("leftLift");
        rightLift = hardwareMap.dcMotor.get ("rightLift");
        servo = hardwareMap.servo.get ("servo");
        servo2 = hardwareMap.servo.get ("servo2");

        //Grab Block
        closeServo();
        //Raise Arm
        //raiseArm (100);
        //Backwards/
         //moveBackwards(1350);
        //turnRight
        //turnRight(450);
        //Forward
       // moveForward(700);
        //dropping block
       // releaseServo();
        //Backwards
      //  moveBackwards(100);
    }

}

