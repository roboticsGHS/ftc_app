package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name="armAUTO", group="armAUTO")
class armAUTO extends OpMode {

    DcMotor leftLift;
    DcMotor rightMotor;
    DcMotor leftMotor;

    public void runOpMode() throws InterruptedException {
        rightMotor = hardwareMap.dcMotor.get("B");
        leftMotor = hardwareMap.dcMotor.get("C");


        //Forward
        leftMotor.setPower(-0.5);
        rightMotor.setPower(-0.5);
        wait(2500);
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        wait(100);

    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
