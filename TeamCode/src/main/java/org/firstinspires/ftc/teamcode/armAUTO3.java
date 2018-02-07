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

@Autonomous(name="armAUTO3", group="armAUTO3")
public class armAUTO3 extends LinearOpMode {

    DcMotor leftMotor;    //backdrive
    DcMotor rightMotor;   //backdrive
    DcMotor leftFront;    //frontdrive
    DcMotor rightFront;   //frontdrive
    DcMotor leftLift;
    DcMotor rightLift;
    DcMotor clawArm;
    Servo servo;  //grabbers
    Servo servo2; //grabbers
    Servo servo3; //color sensor servo
    ModernRoboticsI2cColorSensor colorSensor; //sensor

    //Variables init
    int heading = 0;
    double position = 0.5;

    public void blueJewel () throws InterruptedException {
        servo3.setPosition(0.5);
        boolean isBlue = colorSensor.blue() > 0;
        Thread.sleep(1000);
        if (isBlue)
        {
            moveForward(100);
            Thread.sleep(1000);
            moveBackwards(100);
            Thread.sleep(1000);
        }
        else
        {
            moveBackwards(100);
            Thread.sleep(1000);
            moveForward(100);
            Thread.sleep(1000);

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
        clawArm.setPower(0.5);
        Thread.sleep(time);
    }

    public void lowerArm(int time) throws InterruptedException {
        clawArm.setPower(-0.5);
        Thread.sleep(time);
    }

    public void runOpMode() throws InterruptedException {
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftLift = hardwareMap.dcMotor.get ("leftLift");
        rightLift = hardwareMap.dcMotor.get ("rightLift");
        clawArm = hardwareMap.dcMotor.get("clawArm");
        servo = hardwareMap.servo.get ("servo");
        servo2 = hardwareMap.servo.get ("servo2");
        servo3 = hardwareMap.servo.get ("servo3");
        colorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("colorSensor");


        //Grab Bloc
        closeServo();
        //Raise Arm
        //raiseArm (100);
        //Backwards/
        moveBackwards(700);
        //turnLeft
        turnRight(1000);
        //Strafe
        moveLeft(25);
        //Forward
        moveForward(550);
        openServo();
        //Forward
        moveBackwards(200);
        //red 2 (parallel)//
    }

}

