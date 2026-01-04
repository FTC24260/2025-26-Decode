package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

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
public class Teleop extends OpMode {

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
    public static double kP = 10;
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

    // --- Sensor ignore timer ---
    private long ignoreSensorUntil = 0;
    private static final long SENSOR_IGNORE_MS = 400; // 400 ms after spindex move
    private long initialIgnoreUntil = 0;
    private static final long INITIAL_IGNORE_MS = 500; // ignore first 500 ms after start

    // --- Flicker ---
    private final double flickerUp = 0.5;
    private final double flickerDown = 0.7;

    // --- Intake burst ---
    private long intakeBurstUntil = 0;
    private static final long INTAKE_BURST_MS = 400;

    // --- A toggle / Rapid Fire ---
    private boolean lastA = false;

    private enum RapidFireState {
        IDLE, SPINUP, FLICK, WAIT_NEXT
    }
    private RapidFireState rapidFireState = RapidFireState.IDLE;
    private int rapidFireIndex = 0;
    private long rapidFireTimer = 0;
    private static final long FLICK_UP_MS = 200;
    private static final long WAIT_NEXT_MS = 200;

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

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        DcMotor[] motors = {leftFront, leftRear, rightFront, rightRear};
        for (DcMotor m : motors) m.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");

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
        initialIgnoreUntil = System.currentTimeMillis() + INITIAL_IGNORE_MS; // start 500 ms ignore
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

        // --- Intake logic ---
        if (now < intakeBurstUntil) {
            intake.setPower(-1); // burst
        } else {
            intake.setPower(pipeline.ballDetected ? -1 : 0); // normal intake
        }

        // --- Color detection ---
        String detectedColor = detectColor();

        // Only allow new ball detection if outside initial ignore AND sensor ignore timer
        if (!detectedColor.equals("unknown") && now >= ignoreSensorUntil && now >= initialIgnoreUntil && currentIndex < 3) {
            slots[currentIndex] = detectedColor;
            currentIndex++;
            setSpindexIntakePosition(currentIndex); // move to next slot
            ignoreSensorUntil = now + SENSOR_IGNORE_MS; // ignore further detections for 400 ms
        }

        // --- Shooter PIDF control ---
        double currentVelocity =
                (Math.abs(shooterL.getVelocity()) + Math.abs(shooterR.getVelocity())) / 2.0;

        if (rapidFireState != RapidFireState.IDLE) {
            double ff = feedforward(targetVelocity);
            double fb = feedback(targetVelocity, currentVelocity);
            double power = clamp(ff + fb, 0.0, 1.0);
            shooterL.setPower(power);
            shooterR.setPower(power);
        }

        // --- Rapid Fire FSM ---
        switch (rapidFireState) {

            case SPINUP:
                setSpindexShootPosition(rapidFireIndex);
                if (currentVelocity >= targetVelocity - VELOCITY_TOLERANCE) {
                    flicker.setPosition(flickerUp);
                    rapidFireTimer = now + FLICK_UP_MS;
                    rapidFireState = RapidFireState.FLICK;
                }
                break;

            case FLICK:
                if (now >= rapidFireTimer) {
                    flicker.setPosition(flickerDown);
                    slots[rapidFireIndex] = "unknown";

                    if (rapidFireIndex < slots.length - 1 && anySlotLoaded()) {
                        rapidFireIndex++;
                        intakeBurstUntil = now + INTAKE_BURST_MS;
                        rapidFireTimer = now + WAIT_NEXT_MS;
                        rapidFireState = RapidFireState.WAIT_NEXT;
                    } else {
                        rapidFireState = RapidFireState.IDLE;
                        shooterL.setPower(0);
                        shooterR.setPower(0);
                    }
                }
                break;

            case WAIT_NEXT:
                setSpindexShootPosition(rapidFireIndex);
                if (now >= rapidFireTimer) {
                    rapidFireState = RapidFireState.SPINUP;
                }
                break;

            default:
                break;
        }

        // --- Rapid fire trigger ---
        if (gamepad1.a && !lastA && anySlotLoaded()) {
            rapidFireState = RapidFireState.SPINUP;
            rapidFireIndex = 0;
            rapidFireTimer = 0;
        }
        lastA = gamepad1.a;

        // --- Telemetry ---
        telemetry.addData("Slots", slots[0] + ", " + slots[1] + ", " + slots[2]);
        telemetry.addData("Shooter Vel", currentVelocity);
        telemetry.addData("Rapid Fire State", rapidFireState);
        telemetry.addData("Intake Burst(ms)", Math.max(0, intakeBurstUntil - now));
        telemetry.addData("Sensor Ignore(ms)", Math.max(0, ignoreSensorUntil - now));
        telemetry.addData("Initial Ignore(ms)", Math.max(0, initialIgnoreUntil - now));
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

    private boolean anySlotLoaded() {
        for (String s : slots) {
            if (!s.equals("unknown")) return true;
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
