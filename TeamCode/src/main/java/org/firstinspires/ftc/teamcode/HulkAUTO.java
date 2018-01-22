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
    Servo arm_servo;
    CRServo extend_servo; //grabber

    public void moveForward(int time) throws InterruptedException {
        leftFront.setPower(-0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(0.5);
        Thread.sleep(time);
    }
    public void moveBackwards(int time) throws InterruptedException {
        leftFront.setPower(0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(-0.5);
        Thread.sleep(time);

    }

    public void runOpMode() throws InterruptedException {
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get ("leftBack");
        rightBack = hardwareMap.dcMotor.get ("rightBack");
        liftArm = hardwareMap.dcMotor.get ("liftArm");
        arm_servo = hardwareMap.servo.get ("armServo");
        extend_servo = hardwareMap.crservo.get ("extendServo");

        //Backwards
        moveBackwards(1100);



    }

}

