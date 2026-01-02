package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.ArtifactPipeline;

@TeleOp(name = "Pipeline Intake + Color Spindex")
public class TeleopRestart extends OpMode {

    // --- Vision ---
    private VisionPortal visionPortal;
    private ArtifactPipeline pipeline;

    // --- Hardware ---
    private DcMotor intake;
    private ColorSensor colorSensor;
    private Servo leftIndex, rightIndex;

    // --- Spindex ---
    private final double[] positions = {0.34, 0.603, 1.0};
    private final String[] slots = {"unknown", "unknown", "unknown"};
    private int currentIndex = 0;

    // --- Sensor Lockout ---
    private long ignoreSensorUntil = 0;
    private static final long SENSOR_IGNORE_MS = 400;

    // --- Ball presence tracking ---
    private boolean ballPresentLastLoop = false;

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

        intake.setPower(0);
        setSpindexPosition(0);
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        // --- Intake control ONLY by camera ---
        if (pipeline.ballDetected) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        // --- Color sensor detection ---
        String detectedColor = detectColor();
        boolean colorSeesBall = !detectedColor.equals("unknown");

        // Rising edge: ball JUST arrived at sensor
        boolean ballJustArrived =
                colorSeesBall &&
                        !ballPresentLastLoop &&
                        now >= ignoreSensorUntil &&
                        currentIndex < 3;

        if (ballJustArrived) {
            // Store color ONCE
            slots[currentIndex] = detectedColor;

            // Move spindex
            setSpindexPosition(currentIndex + 1);
            currentIndex++;

            // Lock out sensor
            ignoreSensorUntil = now + SENSOR_IGNORE_MS;
        }

        ballPresentLastLoop = colorSeesBall;

        // --- Telemetry ---
        telemetry.addData("Pipeline Ball Detected", pipeline.ballDetected);
        telemetry.addData("Detected Color", detectedColor);
        telemetry.addData("Slots",
                slots[0] + ", " + slots[1] + ", " + slots[2]);
        telemetry.addData("Current Index", currentIndex);
        telemetry.addData("Ignore Sensor (ms)",
                Math.max(0, ignoreSensorUntil - now));
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0);
        visionPortal.close();
    }

    private String detectColor() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        // Green
        if (g > 1.2 * r && g > 1.2 * b && g > 20) {
            return "green";
        }

        // Purple
        int maxRB = Math.max(r, b);
        int minRB = Math.min(r, b);
        if (maxRB > 40 && minRB >= 0.5 * maxRB && g < 0.7 * maxRB) {
            return "purple";
        }

        return "unknown";
    }

    private void setSpindexPosition(int index) {
        if (index >= positions.length) {
            index = positions.length - 1;
        }
        double pos = positions[index];
        leftIndex.setPosition(pos);
        rightIndex.setPosition(1.0 - pos);
    }
}
