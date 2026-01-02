package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

//@TeleOp(name = "Shooter Test")
public class secondBotShooterTest extends OpMode {
    private DcMotorEx shooter;



    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
    }

    @Override
    public void loop() {
        shooter.setPower(-gamepad1.left_stick_y);
    }
}
