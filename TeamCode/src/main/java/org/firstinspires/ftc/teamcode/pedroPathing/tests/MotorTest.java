package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class MotorTest extends OpMode {

    public DcMotor ShooterL;
    public DcMotor ShooterR;

    public void init() {
        ShooterL = hardwareMap.get(DcMotor.class, "ShooterL");
        ShooterR = hardwareMap.get(DcMotor.class, "ShooterR");

    }

    public void loop() {
        ShooterR.setPower(0.1);
        ShooterL.setPower(0.1);
    }
}
