package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Turret Test", group = "Vision")
public class TurretTest extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor turret;

    private final double deadzone = 4;
    private final double kP = 0.05;
    private final double maxPower = 1;
    private final double minPower = 0.07;

    private final int maxPosition = 430;
    private final int minPosition = -530;

    @Override
    public void runOpMode() throws InterruptedException {

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            double turretPower = 0;
            LLResult result = limelight.getLatestResult();
            int currentPosition = turret.getCurrentPosition();

            if (result != null && result.isValid()) {
                double error = result.getTy();

                if (Math.abs(error) > deadzone) {
                    turretPower = kP * error;

                    // Apply minimum power
                    if (turretPower > 0)
                        turretPower = Math.max(turretPower, minPower);
                    else
                        turretPower = Math.min(turretPower, -minPower);

                    // Clip to max motor power
                    turretPower = Math.max(-maxPower, Math.min(maxPower, turretPower));

                    // Prevent going beyond physical bounds
                    if ((currentPosition >= maxPosition && turretPower > 0) ||
                            (currentPosition <= minPosition && turretPower < 0)) {
                        turretPower = 0;
                    }

                    telemetry.addLine("LOCKED");
                } else {
                    turretPower = 0;
                }

                telemetry.addData("Error", error);

            } else {
                turretPower = 0;
                telemetry.addLine("No target");
            }

            turret.setPower(turretPower);
            telemetry.addData("Power", turretPower);
            telemetry.addData("Position", turret.getCurrentPosition());
            telemetry.update();
        }

        limelight.stop();
    }
}
