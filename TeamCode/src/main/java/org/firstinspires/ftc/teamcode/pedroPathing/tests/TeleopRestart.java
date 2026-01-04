package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.ArtifactPipeline;

@TeleOp(name = "FullTeleOp")
public class TeleopRestart extends OpMode {

    // --- Follower / Drive ---
    private Follower follower;
    public static Pose startingPose;
    private TelemetryManager telemetryM;

    // --- Drive motors ---
    private DcMotor leftFront, leftRear, rightFront, rightRear;

    // --- Vision & intake ---
    private VisionPortal visionPortal;
    private ArtifactPipeline pipeline;
    private DcMotor intake;

    // --- Shooter ---
    private DcMotorEx shooterL, shooterR;

    // --- PIDF constants ---
    public static double kV = 0.00045;
    public static double kS = 0;
    public static double kP = 3;
    public static double targetVelocity = 2200; // ticks/sec
    private static final double VELOCITY_TOLERANCE = 75; // ticks/sec

    // --- Color sensor ---
    private ColorSensor colorSensor;

    // --- Spindex ---
    private Servo leftIndex, rightIndex, flicker;
    private final double[] intakePositions = {0.34, 0.603, 1.0};
    private final double[] shootPositions = {0.73, 0.46, 0.20};
    private final String[] slots = {"unknown", "unknown", "unknown"};
    private int currentIndex = 0;
    private long ignoreSensorUntil = 0;
    private static final long SENSOR_IGNORE_MS = 3000;

    // --- Flicker ---
    private enum FlickerState {IDLE, WAIT_UP, UP}
    private FlickerState flickerState = FlickerState.IDLE;
    private long flickerTimer = 0;
    private final double flickerUp = 0.5;
    private final double flickerDown = 0.7;

    // --- Intake burst ---
    private long intakeBurstUntil = 0;
    private static final long INTAKE_BURST_MS = 500;

    // --- A toggle ---
    private boolean lastA = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        if (startingPose == null) startingPose = new Pose();
        follower.setStartingPose(startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Reverse left motors for proper mecanum
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        DcMotor[] motors = {leftFront, leftRear, rightFront, rightRear};
        for (DcMotor m : motors) m.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");

        // Shooter as DcMotorEx
        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        shooterR.setDirection(DcMotor.Direction.REVERSE);

        pipeline = new ArtifactPipeline();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(pipeline)
                .build();

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");
        flicker = hardwareMap.get(Servo.class, "flicker");

        setSpindexIntakePosition(0);
        flicker.setPosition(flickerDown);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        // --- Mecanum / Teleop drive ---
        follower.update();
        telemetryM.update();
        follower.setTeleOpDrive(
                -gamepad2.left_stick_y,
                -gamepad2.left_stick_x,
                -gamepad2.right_stick_x,
                true
        );

        // --- Intake logic with burst ---
        if (now < intakeBurstUntil) {
            intake.setPower(-1); // burst from A press
        } else {
            intake.setPower(pipeline.ballDetected ? -1 : 0); // normal intake
        }

        // --- Color intake ---
        String detectedColor = detectColor();
        if (!detectedColor.equals("unknown") && now >= ignoreSensorUntil && currentIndex < 3) {
            slots[currentIndex] = detectedColor;
            setSpindexIntakePosition(currentIndex + 1);
            currentIndex++;
            ignoreSensorUntil = now + SENSOR_IGNORE_MS;
        }

        // --- Shooter PIDF control ---
        double currentVelocity =
                (Math.abs(shooterL.getVelocity()) + Math.abs(shooterR.getVelocity())) / 2.0;

        if (flickerState != FlickerState.IDLE) {
            double ff = feedforward(targetVelocity);
            double fb = feedback(targetVelocity, currentVelocity);
            double power = clamp(ff + fb, 0.0, 1.0);

            shooterL.setPower(power);
            shooterR.setPower(power);
        }

        // --- A toggle shooting (velocity based) ---
        if (gamepad1.a && !lastA && shootNearestAny()) {
            // Start spinning shooter, wait until velocity reaches target
            flickerState = FlickerState.WAIT_UP;
            intakeBurstUntil = now + INTAKE_BURST_MS; // burst intake
        }
        lastA = gamepad1.a;

        // --- Flicker FSM ---
        if (flickerState == FlickerState.WAIT_UP) {
            if (currentVelocity >= targetVelocity - VELOCITY_TOLERANCE) {
                flicker.setPosition(flickerUp);
                flickerState = FlickerState.UP;
                flickerTimer = now + 300; // hold up for 300ms
            }
        } else if (flickerState == FlickerState.UP && now >= flickerTimer) {
            flicker.setPosition(flickerDown);
            flickerState = FlickerState.IDLE;
            shooterL.setPower(0);
            shooterR.setPower(0);

            // reset spindex if all slots empty
            if (slots[0].equals("unknown") && slots[1].equals("unknown") && slots[2].equals("unknown")) {
                setSpindexIntakePosition(0);
                currentIndex = 0;
            }
        }

        telemetry.addData("Slots", slots[0] + ", " + slots[1] + ", " + slots[2]);
        telemetry.addData("Flicker State", flickerState);
        telemetry.addData("Shooter Velocity", currentVelocity);
        telemetry.addData("Intake Burst(ms)", Math.max(0, intakeBurstUntil - now));
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0);
        shooterL.setPower(0);
        shooterR.setPower(0);
        visionPortal.close();
    }

    // =======================
    // HELPERS
    // =======================

    private String detectColor() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        if (g > 1.2 * r && g > 1.2 * b && g > 20) return "green";
        int maxRB = Math.max(r, b);
        int minRB = Math.min(r, b);
        if (maxRB > 40 && minRB >= 0.5 * maxRB && g < 0.7 * maxRB) return "purple";
        return "unknown";
    }

    private void setSpindexIntakePosition(int index) {
        if (index >= intakePositions.length) index = intakePositions.length - 1;
        leftIndex.setPosition(intakePositions[index]);
        rightIndex.setPosition(1.0 - intakePositions[index]);
    }

    private void setSpindexShootPosition(int index) {
        if (index >= shootPositions.length) index = shootPositions.length - 1;
        leftIndex.setPosition(shootPositions[index]);
        rightIndex.setPosition(1.0 - shootPositions[index]);
    }

    private boolean shootNearestAny() {
        double currentPos = leftIndex.getPosition();
        double nearestDist = Double.MAX_VALUE;
        int nearestIndex = -1;

        for (int i = 0; i < slots.length; i++) {
            if (!slots[i].equals("unknown")) {
                double dist = Math.abs(currentPos - shootPositions[i]);
                if (dist < nearestDist) {
                    nearestDist = dist;
                    nearestIndex = i;
                }
            }
        }

        if (nearestIndex != -1) {
            setSpindexShootPosition(nearestIndex);
            slots[nearestIndex] = "unknown";
            return true;
        }
        return false;
    }

    // =======================
    // PIDF helpers
    // =======================
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
