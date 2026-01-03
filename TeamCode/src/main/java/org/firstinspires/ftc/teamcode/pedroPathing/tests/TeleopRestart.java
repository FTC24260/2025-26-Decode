package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.ArtifactPipeline;

@TeleOp(name = "FullTeleOp")
public class TeleopRestart extends OpMode {

    // --- Follower ---
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private PathChain pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // --- Drive motors ---
    private DcMotor leftFront, leftRear, rightFront, rightRear;

    // --- Vision & intake ---
    private VisionPortal visionPortal;
    private ArtifactPipeline pipeline;
    private DcMotor intake;

    // --- Shooter ---
    private DcMotor shooterL, shooterR;

    // --- Color sensor ---
    private ColorSensor colorSensor;

    // --- Spindex ---
    private Servo leftIndex, rightIndex, flicker;
    private final double[] intakePositions = {0.34, 0.603, 1.0};
    private final double[] shootPositions = {0.71, 0.46, 0.20};
    private final String[] slots = {"unknown", "unknown", "unknown"};
    private int currentIndex = 0;
    private long ignoreSensorUntil = 0;
    private static final long SENSOR_IGNORE_MS = 1000;

    // --- Flicker ---
    private enum FlickerState {IDLE, WAIT_UP, UP}
    private FlickerState flickerState = FlickerState.IDLE;
    private long flickerTimer = 0;
    private double flickerUp = 0.5;
    private double flickerDown = 0.7;

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

        leftFront.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterL = hardwareMap.get(DcMotor.class, "ShooterL");
        shooterR = hardwareMap.get(DcMotor.class, "ShooterR");

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

        follower.update();
        telemetryM.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );

        // --- Intake ---
        intake.setPower(pipeline.ballDetected ? -1 : 0);

        // --- Color intake ---
        String detectedColor = detectColor();
        if (!detectedColor.equals("unknown") && now >= ignoreSensorUntil && currentIndex < 3) {
            slots[currentIndex] = detectedColor;
            setSpindexIntakePosition(currentIndex + 1);
            currentIndex++;
            ignoreSensorUntil = now + SENSOR_IGNORE_MS;
        }

        // --- A TOGGLE SHOOT (ONLY CHANGE) ---
        if (gamepad1.a && !lastA && shootNearestAny()) {
            startShooting(now);
        }
        lastA = gamepad1.a;

        // --- Flicker FSM ---
        if (flickerState == FlickerState.WAIT_UP && now >= flickerTimer) {
            flicker.setPosition(flickerUp);
            flickerState = FlickerState.UP;
            flickerTimer = now + 400;
        } else if (flickerState == FlickerState.UP && now >= flickerTimer) {
            flicker.setPosition(flickerDown);
            flickerState = FlickerState.IDLE;
            shooterL.setPower(0);
            shooterR.setPower(0);
        }

        telemetry.addData("Slots", slots[0] + ", " + slots[1] + ", " + slots[2]);
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

    // 🔹 NEW: nearest occupied slot (ANY color)
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
            slots[nearestIndex] = "unknown"; // optional but correct
            return true;
        }
        return false;
    }

    private void startShooting(long now) {
        shooterL.setPower(1);
        shooterR.setPower(-1);
        flickerState = FlickerState.WAIT_UP;
        flickerTimer = now + 2000;
        intakeBurstUntil = now + INTAKE_BURST_MS;
    }
}
