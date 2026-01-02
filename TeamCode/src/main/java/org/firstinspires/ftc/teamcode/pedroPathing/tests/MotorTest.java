package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class MotorTest extends OpMode {
    private DcMotor leftFront;

    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "rightFront");
    }

    public void loop() {
        if (gamepad1.a) {
            leftFront.setPower(0.1);
            // LF -
            //LR -
            //
        }
    }
}
