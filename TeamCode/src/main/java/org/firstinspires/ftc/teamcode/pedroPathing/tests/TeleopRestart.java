package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.ArtifactPipeline;

@TeleOp(name = "TeleopRestartFixed")
public class TeleopRestart extends OpMode {

    // --- Vision ---
    private VisionPortal visionPortal;
    private ArtifactPipeline pipeline;

    // --- Hardware ---
    private DcMotor intake;
    private ColorSensor colorSensor;
    private Servo leftIndex, rightIndex, flicker;

    // --- Spindex intake positions ---
    private final double[] intakePositions = {0.34, 0.603, 1.0};

    // --- Spindex shoot positions ---
    private final double[] shootPositions = {0.71, 0.20, 0.46};

    // --- Slot storage ---
    private final String[] slots = {"unknown", "unknown", "unknown"};
    private int currentIndex = 0;

    // --- Sensor Lockout ---
    private long ignoreSensorUntil = 0;
    private static final long SENSOR_IGNORE_MS = 500; // 300ms delay after moving spindex

    // --- Startup delay ---
    private long startupIgnoreUntil = 0;
    private static final long STARTUP_IGNORE_MS = 700;

    // --- Intake burst after shooting ---
    private long intakeBurstUntil = 0;
    private static final long INTAKE_BURST_MS = 300;

    // --- Flicker Positions ---
    private double flickerUp = 0.5;
    private double flickerDown = 0.7;

    // --- Flicker state machine ---
    private enum FlickerState { IDLE, WAIT_UP, UP }
    private FlickerState flickerState = FlickerState.IDLE;
    private long flickerTimer = 0;

    private enum XState { IDLE, EJECTING }
    private XState xState = XState.IDLE;
    private int xCurrentSlot = 0;
    private long xTimer = 0;
    private static final long X_EJECT_MS = 1000;



    @Override
    public void init() {
        pipeline = new ArtifactPipeline();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(pipeline)
                .build();

        intake = hardwareMap.get(DcMotor.class, "intake");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");

        flicker = hardwareMap.get(Servo.class, "flicker");

        intake.setPower(0);
        setSpindexIntakePosition(0);

        flicker.setPosition(flickerDown);

        startupIgnoreUntil = System.currentTimeMillis() + STARTUP_IGNORE_MS;
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        // --- Intake control ---
        if (now < intakeBurstUntil) {
            intake.setPower(-1); // intake burst after shooting
        } else if (pipeline.ballDetected) {
            intake.setPower(-1); // normal intake if ball detected
        } else {
            intake.setPower(0);
        }

        // --- Startup ignore ---
        if (now < startupIgnoreUntil) {
            telemetry.addLine("Color sensor warming up...");
            telemetry.update();
            return;
        }

        // --- Color sensor detection ---
        String detectedColor = detectColor();

        // Only store if:
        // 1. Ball detected by color sensor
        // 2. Not in lockout (ignoreSensorUntil)
        // 3. There is a free slot
        if (!detectedColor.equals("unknown") && now >= ignoreSensorUntil && currentIndex < 3) {
            // Store color in current slot
            slots[currentIndex] = detectedColor;

            // Move spindex to next slot
            setSpindexIntakePosition(currentIndex + 1);

            // Start 300ms lockout before next ball can be stored
            ignoreSensorUntil = now + SENSOR_IGNORE_MS;

            // Advance currentIndex
            currentIndex++;
        }

        // --- Shooting logic ---
        if (gamepad1.a && shootColor("green")) {
            intakeBurstUntil = now + INTAKE_BURST_MS;
            startFlicker(now);
        }

        if (gamepad1.b && shootColor("purple")) {
            intakeBurstUntil = now + INTAKE_BURST_MS;
            startFlicker(now);
        }

        // --- Flicker state machine ---
        if (flickerState == FlickerState.WAIT_UP && now >= flickerTimer) {
            flicker.setPosition(flickerUp);
            flickerState = FlickerState.UP;
            flickerTimer = now + 300; // hold up for 100ms
        } else if (flickerState == FlickerState.UP && now >= flickerTimer) {
            flicker.setPosition(flickerDown);
            flickerState = FlickerState.IDLE;
        }

        // --- X button eject/reset logic ---
        if (gamepad1.x && xState == XState.IDLE) {
            xState = XState.EJECTING;
            xCurrentSlot = 0;
            setSpindexIntakePosition(xCurrentSlot);
            intake.setPower(1); // eject
            xTimer = System.currentTimeMillis() + X_EJECT_MS;
        }

        if (xState == XState.EJECTING) {
            now = System.currentTimeMillis();
            if (now >= xTimer) {
                xCurrentSlot++;
                if (xCurrentSlot < intakePositions.length) {
                    setSpindexIntakePosition(xCurrentSlot);
                    intake.setPower(1); // eject next ball
                    xTimer = now + X_EJECT_MS;
                } else {
                    intake.setPower(0); // done ejecting
                    xState = XState.IDLE;
                }
            }
        }


        // --- Telemetry ---
        telemetry.addData("Detected Color", detectedColor);
        telemetry.addData("Slots", slots[0] + ", " + slots[1] + ", " + slots[2]);
        telemetry.addData("Current Index", currentIndex);
        telemetry.addData("Ball Detected by Intake", pipeline.ballDetected);
        telemetry.addData("Sensor Lockout(ms)", Math.max(0, ignoreSensorUntil - now));
        telemetry.addData("Intake Burst(ms)", Math.max(0, intakeBurstUntil - now));
        telemetry.addData("Flicker State", flickerState);
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0);
        visionPortal.close();
    }

    // =======================
    // HELPER METHODS
    // =======================

    private boolean shootColor(String color) {
        for (int i = 0; i < 3; i++) {
            if (slots[i].equals(color)) {
                setSpindexShootPosition(i);
                return true;
            }
        }
        return false;
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

    private void startFlicker(long now) {
        flickerState = FlickerState.WAIT_UP;
        flickerTimer = now + 700; // wait 400ms before flicker up
    }
}
