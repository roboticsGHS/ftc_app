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

@Autonomous(name="armAUTO2", group="armAUTO2")
public class armAUTO2 extends LinearOpMode {

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
    //ColorSensor colorSensor; //sensor
    ColorSensor revColor;


    //Variables init
    int heading = 0;
    double position = 1;

    public void blueJewel() throws InterruptedException {
        servo3.setPosition(0);
        RobotLog.a("**************************************** REV COLOR READING - red: " + revColor.red() + " -- blue: " + revColor.blue());
        boolean isBlue = revColor.blue() > revColor.red();
        Thread.sleep(3000);
        RobotLog.a("**************************************** REV COLOR READING - red: " + revColor.red() + " -- blue: " + revColor.blue());
        if (revColor.red() > revColor.blue()) {
            turnRight(50);
            //  Thread.sleep(1000);
            servo3.setPosition(0.5);
            //  Thread.sleep(1000);
            turnLeft(50);
            //  Thread.sleep(1000);
        } else {
            turnLeft(50);
            // Thread.sleep(1000);
            raiseSensor();
            //  Thread.sleep(1000);
            turnRight(50);
            //   Thread.sleep(1000);
            //  moveForward(50);
            //   Thread.sleep(1000);

        }
    }

    public void moveForward(int time) throws InterruptedException {
        leftMotor.setPower(-0.3);
        rightMotor.setPower(0.3);
        leftFront.setPower(-0.3);
        rightFront.setPower(0.3);
        Thread.sleep(time);
    }

    public void moveRight(int time) throws InterruptedException {
        leftLift.setPower(0.3);
        rightLift.setPower(-0.3);
        Thread.sleep(time);
    }

    public void moveLeft(int time) throws InterruptedException {
        leftLift.setPower(-0.3);
        rightLift.setPower(0.3);
        Thread.sleep(time);
    }

    public void moveBackwards(int time) throws InterruptedException {
        leftMotor.setPower(0.3);
        rightMotor.setPower(-0.3);
        leftFront.setPower(0.3);
        rightFront.setPower(-0.3);
        Thread.sleep(time);
    }

    public void turnRight(int time) throws InterruptedException {
        rightMotor.setPower(-0.3);
        rightFront.setPower(-0.3);
        leftMotor.setPower(-0.3);
        leftFront.setPower(-0.3);
        Thread.sleep(time);
    }

    public void turnLeft(int time) throws InterruptedException {
        rightMotor.setPower(0.3);
        rightFront.setPower(0.3);
        leftMotor.setPower(0.3);
        leftFront.setPower(0.3);
        Thread.sleep(time);
    }

    public void openServo() {
        servo.setPosition(0.9); //left servo needs to be lower number to open
        servo2.setPosition(0.1); //right servo needs to be a higher number to open
    }

    public void closeServo() {
        servo.setPosition(0.1); //left servo needs to be higher number to close
        servo2.setPosition(0.9); //right servo needs to be a low number to close
    }

    public void raiseArm(int time) throws InterruptedException {
        clawArm.setPower(0.25);
        Thread.sleep(time);
    }

    public void lowerArm(int time) throws InterruptedException {
        clawArm.setPower(-0.3);
        Thread.sleep(time);
    }

    public void raiseSensor() {
        servo3.setPosition(0.9);
    }

    public void waitTime (int time) throws InterruptedException {
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
        //colorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("colorSensor");
        //colorSensor = hardwareMap.colorSensor.get("colorSensor");
        revColor = hardwareMap.colorSensor.get("revColor");


        //Grab Bloc
        waitForStart();
        closeServo();
        //Raise Arm
        raiseArm (100);
        //wait
        waitTime(1000);
        //Backwards/
        //blueJewel();
        moveBackwards(2500);
        //turnLeft
        turnLeft(50);
        //Forward
        moveForward(175);
        lowerArm(100);
        //wait
        waitTime(1000);
        //open servo
        openServo();
        //Forward
        moveBackwards(350);
        //red//
    }

}

