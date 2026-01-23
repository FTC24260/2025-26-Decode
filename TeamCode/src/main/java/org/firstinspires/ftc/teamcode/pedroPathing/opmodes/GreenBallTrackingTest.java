package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Green Ball Follower", group = "Vision")
public class GreenBallTrackingTest extends LinearOpMode {

    private Limelight3A limelight;

    private DcMotor leftFront, rightFront, leftRear, rightRear;

    // Tunable proportional gains
    private final double kP_turn = 0.02;
    private final double kP_drive = 0.01;

    // Target area calibration for ~0.5 ft distance
    private double targetTa = 25;

    // Minimum target area to ignore background noise
    private final double MIN_TA = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(3); // green ball pipeline

        telemetry.addLine("Initialization complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                // No target found
                setMotorPower(0,0,0,0);
                telemetry.addData("Status", "No target detected");
                telemetry.update();
                sleep(50);
                continue;
            }

            double tx = result.getTx();  // horizontal offset
            double ta = result.getTa();  // target area (%)

            // Deadzone: ignore tiny noises
            if (ta < MIN_TA) {
                setMotorPower(0,0,0,0);
                telemetry.addData("Status", "Target too small, ignoring");
                telemetry.update();
                sleep(20);
                continue;
            }

            // --- Compute turn and forward/back ---
            double turn = Range.clip(-kP_turn * tx, -0.4, 0.4); // flip sign to correct direction
            double driveError = targetTa - ta;                   // positive -> move forward
            double drive = Range.clip(kP_drive * driveError, -0.5, 0.5);

            double strafe = 0; // no strafing

            // --- Mecanum drive calculation ---
            double flPower = drive + strafe + turn;
            double frPower = drive - strafe - turn;
            double blPower = drive - strafe + turn;
            double brPower = drive + strafe - turn;

            setMotorPower(flPower, frPower, blPower, brPower);

            // --- Telemetry ---
            telemetry.addData("tx (deg)", "%.2f", tx);
            telemetry.addData("ta (%)", "%.3f", ta);
            telemetry.addData("drive", "%.3f", drive);
            telemetry.addData("turn", "%.3f", turn);
            telemetry.update();

            sleep(20);
        }

        limelight.stop();
    }

    // Helper method to set all four motor powers
    private void setMotorPower(double fl, double fr, double bl, double br) {
        leftFront.setPower(Range.clip(fl, -1, 1));
        rightFront.setPower(Range.clip(fr, -1, 1));
        leftRear.setPower(Range.clip(bl, -1, 1));
        rightRear.setPower(Range.clip(br, -1, 1));
    }
}
