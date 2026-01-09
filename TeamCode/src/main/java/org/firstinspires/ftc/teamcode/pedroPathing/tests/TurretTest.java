package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Turret Test", group = "Vision")
public class TurretTest extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor turret;

    // ===== TUNING =====
    private final double kP = 0.02;
    private final double maxPower = 0.4;
    private final double deadzone = 1.0; // degrees

    @Override
    public void runOpMode() throws InterruptedException {

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.addLine("Starting Limelight...");
        telemetry.update();

        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            double turretPower = 0;
            LLResult result = null;

            // ===== FIND WHICH PIPELINE SEES A VALID TARGET =====
            for (int pipeline = 0; pipeline <= 2; pipeline++) {
                limelight.pipelineSwitch(pipeline);
                sleep(40); // allow pipeline to update

                LLResult tempResult = limelight.getLatestResult();
                if (tempResult != null && tempResult.isValid()) {
                    result = tempResult;
                    break;
                }
            }

            // ===== IF WE SEE A TARGET =====
            if (result != null && result.isValid()) {

                double tx = result.getTx(); // horizontal offset

                if (Math.abs(tx) > deadzone) {
                    turretPower = tx * kP;
                    turretPower = Math.max(-maxPower,
                            Math.min(maxPower, turretPower));
                } else {
                    turretPower = 0; // LOCKED
                }

                telemetry.addLine("LOCKED ON TARGET");
                telemetry.addData("tx", tx);

            } else {
                turretPower = 0;
                telemetry.addLine("No target detected");
            }

            turret.setPower(turretPower);
            telemetry.addData("Turret Power", turretPower);
            telemetry.update();
        }

        limelight.stop();
    }
}
