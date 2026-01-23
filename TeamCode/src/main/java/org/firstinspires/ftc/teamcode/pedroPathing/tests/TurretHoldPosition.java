package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name = "TurretHoldPosition")
public class TurretHoldPosition extends OpMode {

    private DcMotor turret;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setDirection(DcMotor.Direction.REVERSE);

        // Treat current position as 0
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.2); // small power to hold position
    }

    @Override
    public void loop() {
        // Just keep the turret at tick 0
        turret.setTargetPosition(-470);  //550  //-470
        telemetry.addData("Turret Position", turret.getCurrentPosition());
        telemetry.update();
    }
}
