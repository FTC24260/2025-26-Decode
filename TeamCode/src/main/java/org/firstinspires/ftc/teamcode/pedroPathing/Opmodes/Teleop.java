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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.ArtifactPipeline;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class Teleop extends OpMode {

    // --- Follower stuff ---
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // --- Vision & intake stuff ---
    private VisionPortal visionPortal;
    private ArtifactPipeline pipeline;
    private DcMotor intake;

    // --- Color sensor ---
    private ColorSensor colorSensor;

    @Override
    public void init() {
        // --- Follower initialization ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        // --- Vision & intake initialization ---
        pipeline = new ArtifactPipeline();
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setPower(0);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(pipeline)
                .build();

        // --- Color sensor initialization ---
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        telemetry.addLine("TeleOp with Color Sensor Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // --- Follower teleop logic ---
        follower.update();
        telemetryM.update();

        // Teleop driving
        double lx = -gamepad1.left_stick_y;
        double ly = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        if (slowMode) {
            lx *= slowModeMultiplier;
            ly *= slowModeMultiplier;
            rx *= slowModeMultiplier;
        }

        follower.setTeleOpDrive(lx, ly, rx, false); // Robot Centric

        // Automated path following
        if (gamepad1.a) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.b || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // Slow mode toggles
        if (gamepad1.right_bumper) slowMode = !slowMode;
        if (gamepad1.x) slowModeMultiplier += 0.25;
        if (gamepad2.y) slowModeMultiplier -= 0.25;

        // --- Vision-based intake logic ---
        if (pipeline.ballDetected) {
            intake.setPower(-1); // run intake if green detected
        } else {
            intake.setPower(0);
        }

        // --- Color sensor detection (green/purple only) ---
        String detectedColor = detectColor();
        telemetry.addData("Detected Color", detectedColor);
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

        // Green: significantly higher than R and B
        if (g > 1.2 * r && g > 1.2 * b) return "green";

        // Purple: red + blue high, green low
        if (r > 50 && b > 50 && g < 40) return "purple";

        return "unknown";
    }
}
