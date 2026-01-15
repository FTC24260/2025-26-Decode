package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.ArtifactPipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.ShooterSubsystem;

@TeleOp(name = "FullTeleOp")
public class Teleop extends OpMode {

    RobotConstants robotConstant;
    DriveSubsystem driveSubsystem;
    ShooterSubsystem shooterSubsystem;
    private TelemetryManager telemetryM;

    private VisionPortal visionPortal;
    private ArtifactPipeline pipeline;
    private DcMotor intake;

    public static double targetVelocity = 100;

    private ColorSensor colorSensor;

    private Servo leftIndex, rightIndex, flicker;

    private final double[] intakePositions = {0.55, 0.813, 0.3};
    private final double[] shootPositions  = {0.68, 0.42, 0.17};

    private final String[] slots = {"unknown", "unknown", "unknown"};
    private int currentIndex = 0;

    private long ignoreSensorUntil = 0;
    private static final long SENSOR_IGNORE_MS = 1000;

    private long initialIgnoreUntil = 0;
    private static final long INITIAL_IGNORE_MS = 1500;

    private long intakeBurstUntil = 0;
    private static final long INTAKE_BURST_MS = 800;

    private final double flickerUp = 0.5;
    private final double flickerDown = 0.75;

    private enum RapidFireState {
        IDLE,
        SPINUP_WAIT,
        FLICK_UP
    }

    private RapidFireState rapidFireState = RapidFireState.IDLE;
    private int rapidFireIndex = 0;
    private long rapidFireTimer = 0;

    private static final long SPINUP_DELAY_MS = 1000;
    private static final long FLICK_UP_MS = 200;

    private boolean lastA = false;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");

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
        initialIgnoreUntil = System.currentTimeMillis() + INITIAL_IGNORE_MS;
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        double y = -gamepad2.left_stick_y;
        double x = gamepad2.left_stick_x;
        double rx = gamepad2.right_stick_x;

        driveSubsystem.teleopDrive(y, x, rx);
        driveSubsystem.update();

        if (now < intakeBurstUntil) {
            intake.setPower(-1);
        } else {
            intake.setPower(pipeline.ballDetected ? -1 : 0);
        }

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

        if (rapidFireState != RapidFireState.IDLE) {
            shooterSubsystem.setTargetVelocity(targetVelocity);
        }

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
                        rapidFireState = RapidFireState.IDLE;
                        shooterSubsystem.stopShooter();
                        rapidFireIndex = 0;
                        currentIndex = 0;
                        setSpindexIntakePosition(0);
                    }
                }
                break;
        }

        if (gamepad1.a && !lastA && anySlotLoaded()) {
            intakeBurstUntil = now + INTAKE_BURST_MS;
            rapidFireIndex = 0;
            rapidFireTimer = now + SPINUP_DELAY_MS;
            rapidFireState = RapidFireState.SPINUP_WAIT;
        }
        lastA = gamepad1.a;

        telemetry.addData("Slots", slots[0] + ", " + slots[1] + ", " + slots[2]);
        telemetry.addData("RapidFire", rapidFireState);
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0);
        shooterSubsystem.stopShooter();
        visionPortal.close();
    }

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
}
