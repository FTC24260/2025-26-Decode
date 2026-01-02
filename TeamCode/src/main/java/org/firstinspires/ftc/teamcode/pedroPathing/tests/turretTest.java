package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp
public class turretTest extends OpMode {

    private DcMotor turret;
    double turretPower = 0.5;
    double idlePower = 0.0;

    public void init() {
        turret = hardwareMap.get(DcMotor.class, "turret");

    }

    public void loop() {
        if (gamepad1.left_trigger > 0.1) {
            turret.setPower(turretPower);
        }
        else if (gamepad1.right_trigger > 0.1) {
            turret.setPower(-turretPower);
        }
        else {
            turret.setPower(idlePower);
        }

    }
}
