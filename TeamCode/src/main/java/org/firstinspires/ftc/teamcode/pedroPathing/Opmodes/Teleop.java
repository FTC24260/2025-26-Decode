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

    // =======================
    // DRIVE
    // =======================
    private Follower follower;
    public static Pose startingPose;
    private TelemetryManager telemetryM;

    private DcMotor leftFront, leftRear, rightFront, rightRear;

    // =======================
    // VISION / INTAKE
    // =======================
    private VisionPortal visionPortal;
    private ArtifactPipeline pipeline;
    private DcMotor intake;

    // =======================
    // SHOOTER
    // =======================
    private DcMotorEx shooterL, shooterR;

    public static double kV = 0.0005;
    public static double kS = 0.4;
    public static double targetVelocity = 2200;

    // =======================
    // COLOR SENSOR
    // =======================
    private ColorSensor colorSensor;

    // =======================
    // SPINDEX
    // =======================
    private Servo leftIndex, rightIndex, flicker;

    private final double[] intakePositions = {0.34, 0.603, 1.0};
    private final double[] shootPositions  = {0.73, 0.46, 0.20};

    private final String[] slots = {"unknown", "unknown", "unknown"};
    private int currentIndex = 0;

    // =======================
    // TIMERS
    // =======================
    private long ignoreSensorUntil = 0;
    private static final long SENSOR_IGNORE_MS = 450;

    private long initialIgnoreUntil = 0;
    private static final long INITIAL_IGNORE_MS = 900;

    private long intakeBurstUntil = 0;
    private static final long INTAKE_BURST_MS = 400;

    // =======================
    // FLICKER
    // =======================
    private final double flickerUp = 0.5;
    private final double flickerDown = 0.7;

    // =======================
    // RAPID FIRE FSM
    // =======================
    private enum RapidFireState {
        IDLE,
        SPINUP_WAIT,
        FLICK_UP
    }

    private RapidFireState rapidFireState = RapidFireState.IDLE;
    private int rapidFireIndex = 0;
    private long rapidFireTimer = 0;

    private static final long SPINUP_DELAY_MS = 900;
    private static final long FLICK_UP_MS = 200;

    private boolean lastA = false;

    // =======================
    // INIT
    // =======================
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        if (startingPose == null) startingPose = new Pose();
        follower.setStartingPose(startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        }

        intake = hardwareMap.get(DcMotor.class, "intake");

        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        shooterR.setDirection(DcMotor.Direction.REVERSE);

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);

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
        initialIgnoreUntil = System.currentTimeMillis() + INITIAL_IGNORE_MS;
    }

    // =======================
    // LOOP
    // =======================
    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        // --- Drive ---
        follower.update();
        telemetryM.update();
        follower.setTeleOpDrive(
                -gamepad2.left_stick_y,
                -gamepad2.left_stick_x,
                -gamepad2.right_stick_x,
                true
        );

        // --- Intake ---
        if (now < intakeBurstUntil) {
            intake.setPower(-1);
        } else {
            intake.setPower(pipeline.ballDetected ? -1 : 0);
        }

        // --- Color detect ---
        String detectedColor = detectColor();
        if (!detectedColor.equals("unknown")
                && now >= ignoreSensorUntil
                && now >= initialIgnoreUntil
                && currentIndex < 3) {

            slots[currentIndex] = detectedColor;
            currentIndex++;
            setSpindexIntakePosition(currentIndex);
            ignoreSensorUntil = now + SENSOR_IGNORE_MS;
        }

        // =======================
        // SHOOTER (MAX POWER ENTIRE BURST)
        // =======================
        if (rapidFireState != RapidFireState.IDLE) {
            double power = clamp(feedforward(targetVelocity), 0, 1);
            shooterL.setPower(power);
            shooterR.setPower(power);
        }

        // =======================
        // RAPID FIRE FSM
        // =======================
        switch (rapidFireState) {

            case SPINUP_WAIT:
                setSpindexShootPosition(rapidFireIndex);
                if (now >= rapidFireTimer) {
                    flicker.setPosition(flickerUp);
                    rapidFireTimer = now + FLICK_UP_MS;
                    rapidFireState = RapidFireState.FLICK_UP;
                }
                break;

            case FLICK_UP:
                if (now >= rapidFireTimer) {
                    flicker.setPosition(flickerDown);
                    slots[rapidFireIndex] = "unknown";

                    if (rapidFireIndex < 2 && anySlotLoaded()) {
                        rapidFireIndex++;
                        intakeBurstUntil = now + INTAKE_BURST_MS;
                        rapidFireTimer = now + SPINUP_DELAY_MS;
                        rapidFireState = RapidFireState.SPINUP_WAIT;
                    } else {
                        // DONE — reset everything
                        rapidFireState = RapidFireState.IDLE;
                        shooterL.setPower(0);
                        shooterR.setPower(0);

                        rapidFireIndex = 0;
                        currentIndex = 0;
                        setSpindexIntakePosition(0);
                    }
                }
                break;

            default:
                break;
        }

        // =======================
        // TRIGGER
        // =======================
        if (gamepad1.a && !lastA && anySlotLoaded()) {
            rapidFireIndex = 0;
            rapidFireTimer = now + SPINUP_DELAY_MS;
            rapidFireState = RapidFireState.SPINUP_WAIT;
        }
        lastA = gamepad1.a;

        // =======================
        // TELEMETRY
        // =======================
        telemetry.addData("Slots", slots[0] + ", " + slots[1] + ", " + slots[2]);
        telemetry.addData("RapidFire", rapidFireState);
        telemetry.addData("CurrentIndex", currentIndex);
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

    private double feedforward(double targetVel) {
        return kS * Math.signum(targetVel) + kV * targetVel;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
