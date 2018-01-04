package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@Autonomous(name="HulkAUTO", group="HulkAUTO")
public class HulkAUTO extends LinearOpMode {

    DcMotor leftFront;    //drive
    DcMotor rightFront;   //drive
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor liftArm;
    Servo grabber;
    CRServo extend_servo; //grabber

    public void moveForward(int time) throws InterruptedException {
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
        Thread.sleep(time);
    }
    public void moveBackwards(int time) throws InterruptedException {
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(-0.5);
        Thread.sleep(time);

    }

    public void runOpMode() throws InterruptedException {
        rightFront = hardwareMap.dcMotor.get("rightMotor");
        leftFront = hardwareMap.dcMotor.get("leftMotor");
        leftBack = hardwareMap.dcMotor.get ("leftLift");
        rightBack = hardwareMap.dcMotor.get ("rightLift");
        grabber = hardwareMap.servo.get ("grabber");
        extend_servo = hardwareMap.crservo.get ("extendServo");

        //Backwards
        moveBackwards(2500);



    }

}

