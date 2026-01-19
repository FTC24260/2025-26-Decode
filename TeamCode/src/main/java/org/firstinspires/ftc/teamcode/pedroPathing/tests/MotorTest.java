package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class MotorTest extends OpMode {

    public DcMotor testMotor;

    public void init() {
        testMotor = hardwareMap.get(DcMotor.class, "rightFront");

    }

    public void loop() {
        testMotor.setPower(-0.3);

    }
}
