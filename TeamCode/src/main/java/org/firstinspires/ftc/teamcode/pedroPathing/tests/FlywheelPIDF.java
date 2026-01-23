package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//@TeleOp(name = "Shooter PIDF Tuner")
public class FlywheelPIDF extends OpMode {

    // Shooter motors
    private DcMotorEx shooterL;
    private DcMotorEx shooterR;

    // Tunable constants
    public static double kV = 0.00045;
    public static double kS = 0;
    public static double kP = 3;
    public static double targetVelocity = 2200; // ticks/sec - 2200

    // Step sizes
    private static final double KV_STEP = 0.00001;
    private static final double KS_STEP = 0.001;
    private static final double KP_STEP = 0.00005;
    private static final double VEL_STEP = 50;

    // Button edge detection
    private boolean lastUp, lastDown, lastLeft, lastRight;
    private boolean lastLB, lastRB, lastA, lastB;

    @Override
    public void init() {
        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reverse ONE motor if wheels oppose
        shooterR.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        // =====================
        // DPAD / BUTTON TUNING
        // =====================

        // Target velocity
        if (gamepad1.dpad_up && !lastUp) targetVelocity += VEL_STEP;
        if (gamepad1.dpad_down && !lastDown) targetVelocity -= VEL_STEP;

        // kP
        if (gamepad1.dpad_right && !lastRight) kP += KP_STEP;
        if (gamepad1.dpad_left && !lastLeft) kP -= KP_STEP;

        // kV
        if (gamepad1.right_bumper && !lastRB) kV += KV_STEP;
        if (gamepad1.left_bumper && !lastLB) kV -= KV_STEP;

        // kS
        if (gamepad1.a && !lastA) kS += KS_STEP;
        if (gamepad1.b && !lastB) kS -= KS_STEP;

        // Clamp safety
        kP = Math.max(0, kP);
        kV = Math.max(0, kV);
        kS = Math.max(0, kS);
        targetVelocity = Math.max(0, targetVelocity);

        // =====================
        // CONTROL LOOP
        // =====================

        double currentVelocity =
                (Math.abs(shooterL.getVelocity()) + Math.abs(shooterR.getVelocity())) / 2.0;

        double ff = feedforward(targetVelocity);
        double fb = feedback(targetVelocity, currentVelocity);

        double power = clamp(ff + fb, 0.0, 1.0);

        shooterL.setPower(power);
        shooterR.setPower(power);

        // =====================
        // TELEMETRY
        // =====================

        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Current Vel", currentVelocity);
        telemetry.addData("Error", targetVelocity - currentVelocity);
        telemetry.addData("Power", power);

        telemetry.addLine("DPAD ↑↓ : Target Velocity");
        telemetry.addLine("DPAD ←→ : kP");
        telemetry.addLine("LB / RB : kV");
        telemetry.addLine("A / B   : kS");

        telemetry.update();

        // =====================
        // Save last states
        // =====================
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastLB = gamepad1.left_bumper;
        lastRB = gamepad1.right_bumper;
        lastA = gamepad1.a;
        lastB = gamepad1.b;
    }

    private double feedforward(double targetVel) {
        if (Math.abs(targetVel) < 1e-6) return 0;
        return kS * Math.signum(targetVel) + kV * targetVel;
    }

    private double feedback(double targetVel, double currentVel) {
        return kP * (targetVel - currentVel);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
