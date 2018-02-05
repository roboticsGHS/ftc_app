package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

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
    Servo servo3; //color sensor servo
    ModernRoboticsI2cColorSensor colorSensor; //sensor

    //Variables init
    int heading = 0;
    double position = 0.5;

    public void blueJewel (int time) throws InterruptedException {
        servo3.setPosition(0.0);
        boolean isBlue = colorSensor.blue() > 0;
        if (isBlue)
        {
          //  moveForward(2000);
            closeServo();
        }
        else
        {
            //moveBackwards(1000);
            openServo();
        }
        servo3.setPosition(1);
    }

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

    public void moveLeft(int time) throws InterruptedException {
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

    public void turnRight(int time) throws InterruptedException {
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

    public void openServo() {
        servo.setPosition(0.2); //left servo needs to be lower number to open
        servo2.setPosition(0.8); //right servo needs to be a higher number to open
    }

    public void closeServo() {
        servo.setPosition(0.9); //left servo needs to be higher number to close
        servo2.setPosition(0.1); //right servo needs to be a low number to close
    }

    public void raiseArm(int time) throws InterruptedException {
        ClawArm.setPower(0.5);
        Thread.sleep(time);
    }

    public void lowerArm(int time) throws InterruptedException {
        ClawArm.setPower(-0.5);
        Thread.sleep(time);
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
        servo3 = hardwareMap.servo.get ("servo3");
        colorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("colorSensor");

        //Grab Bloc
        openServo();/*
        //Raise Arm
        //raiseArm (100);
        //Backwards/
         moveBackwards(1400);
        //turnRight
        turnRight(500);
        //Forward
        moveForward(600);
        //dropping block
        openServo();
        //Backwards
        moveBackwards(200);
        //blue//
        */
        blueJewel(100);
    }

}

