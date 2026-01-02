package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.ArtifactPipeline;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class Teleop extends OpMode {

    // --- Follower ---
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // --- Vision & intake ---
    private VisionPortal visionPortal;
    private ArtifactPipeline pipeline;
    private DcMotor intake;

    // --- Color sensor ---
    private ColorSensor colorSensor;

    // --- Spindex ---
    private Servo leftIndex, rightIndex;
    private final double[] positions = {0.34, 0.603, 1.0};
    private final String[] slots = {"unknown", "unknown", "unknown"};
    private int currentIndex = 0; // next empty slot
    private long ignoreSensorUntil = 0; // delay until next ball can be read
    private final long SENSOR_IGNORE_MS = 400; // 400ms delay after moving spindex

    // --- Ball stabilization ---
    private long ballDetectedTime = 0; // when ball first detected
    private final long BALL_STABLE_MS = 80; // wait 80ms for ball to settle

    // --- Final intake after 3 balls ---
    private boolean finalMoveStarted = false;
    private long finalIntakeStartTime = 0;
    private final long FINAL_INTAKE_MS = 500;

    @Override
    public void init() {
        // --- Follower ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        // --- Vision & intake ---
        pipeline = new ArtifactPipeline();
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setPower(0);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(pipeline)
                .build();

        // --- Color sensor ---
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // --- Spindex ---
        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");
        currentIndex = 0;
        setSpindexPosition(currentIndex);

        telemetry.addLine("TeleOp with ArtifactPipeline & Spindex Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        // --- Driving ---
        follower.update();
        telemetryM.update();

        double lx = -gamepad1.left_stick_y;
        double ly = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        if (slowMode) {
            lx *= slowModeMultiplier;
            ly *= slowModeMultiplier;
            rx *= slowModeMultiplier;
        }
        follower.setTeleOpDrive(lx, ly, rx, false);

        // --- Automated path following ---
        if (gamepad1.a) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        if (automatedDrive && (gamepad1.b || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // --- Slow mode toggles ---
        if (gamepad1.right_bumper) slowMode = !slowMode;
        if (gamepad1.x) slowModeMultiplier += 0.25;
        if (gamepad2.y) slowModeMultiplier -= 0.25;

        // --- Vision-based intake ---
        if (pipeline.ballDetected) {
            intake.setPower(-1);

            // Ball stabilization
            if (ballDetectedTime == 0) {
                ballDetectedTime = now; // first loop ball is seen
            }

            if (now - ballDetectedTime >= BALL_STABLE_MS &&
                    now >= ignoreSensorUntil &&
                    currentIndex < 3) {

                String detectedColor = detectColor();
                if (detectedColor.equals("green") || detectedColor.equals("purple")) {
                    // Store the color in current slot
                    slots[currentIndex] = detectedColor;

                    // Move spindex to next slot
                    setSpindexPosition(currentIndex + 1);

                    // Advance currentIndex
                    currentIndex++;

                    // Start ignore timer to prevent double-detection
                    ignoreSensorUntil = now + SENSOR_IGNORE_MS;

                    // Reset ballDetectedTime for next ball
                    ballDetectedTime = 0;
                }
            }

        } else {
            intake.setPower(0);
            ballDetectedTime = 0; // reset when no ball detected
        }

        // --- After 3 balls collected ---
        if (currentIndex == 3 && !finalMoveStarted) {
            setSpindexPosition(0); // move to first slot
            intake.setPower(-1);   // run intake for 500ms
            finalMoveStarted = true;
            finalIntakeStartTime = now;
        }

        // Stop intake after final move
        if (finalMoveStarted && now - finalIntakeStartTime >= FINAL_INTAKE_MS) {
            intake.setPower(0);
        }

        // --- Telemetry ---
        telemetry.addData("Detected Color", detectColor());
        telemetry.addData("Slots", slots[0] + ", " + slots[1] + ", " + slots[2]);
        telemetry.addData("Next Slot Index", currentIndex);
        telemetry.addData("Ball Detected by Intake", pipeline.ballDetected);
        telemetry.addData("Sensor Ignore(ms)", Math.max(0, ignoreSensorUntil - now));
        telemetry.update();
    }

    @Override
    public void stop() {
        visionPortal.close();
        intake.setPower(0);
    }

    private String detectColor() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        // Green detection
        if (g > 1.2 * r && g > 1.2 * b && g > 20) return "green";

        // Purple detection (broad)
        int maxRB = Math.max(r, b);
        int minRB = Math.min(r, b);
        boolean enoughRedBlue = maxRB > 40;
        boolean balancedRedBlue = minRB >= 0.5 * maxRB;
        boolean lowGreen = g < 0.7 * maxRB;
        if (enoughRedBlue && balancedRedBlue && lowGreen) return "purple";

        return "unknown";
    }

    private void setSpindexPosition(int index) {
        if (index >= positions.length) index = positions.length - 1;
        double pos = positions[index];
        leftIndex.setPosition(pos);
        rightIndex.setPosition(1.0 - pos);
    }
}
