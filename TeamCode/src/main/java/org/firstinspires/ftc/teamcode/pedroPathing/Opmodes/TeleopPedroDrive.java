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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleopPedroDrive extends OpMode {

    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    public static double joystickDeadzone = 0.05;
    public static double brakeGain = 2;
    public static double maxBrakePower = 1;

    private Pose lastPose = new Pose();
    private long lastTimeNs;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(45),
                                0.8
                        )
                )
                .build();

        lastPose = follower.getPose();
        lastTimeNs = System.nanoTime();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        lastTimeNs = System.nanoTime();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        Pose currentPose = follower.getPose();
        long now = System.nanoTime();
        double dt = (now - lastTimeNs) / 1e9;

        double vx = 0;
        double vy = 0;
        double vHeading = 0;

        if (dt > 0) {
            vx = (currentPose.getX() - lastPose.getX()) / dt;
            vy = (currentPose.getY() - lastPose.getY()) / dt;
            vHeading = (currentPose.getHeading() - lastPose.getHeading()) / dt;
        }

        lastPose = currentPose;
        lastTimeNs = now;

        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double r = -gamepad1.right_stick_x;

        boolean sticksZero =
                Math.abs(y) < joystickDeadzone &&
                        Math.abs(x) < joystickDeadzone &&
                        Math.abs(r) < joystickDeadzone;

        if (!automatedDrive) {

            if (sticksZero) {
                double brakeY = clamp(-vx * brakeGain, -maxBrakePower, maxBrakePower);
                double brakeX = clamp(-vy * brakeGain, -maxBrakePower, maxBrakePower);
                double brakeR = clamp(-vHeading * brakeGain, -maxBrakePower, maxBrakePower);

                follower.setTeleOpDrive(
                        brakeY,
                        brakeX,
                        brakeR,
                        true
                );
            } else {
                if (slowMode) {
                    y *= slowModeMultiplier;
                    x *= slowModeMultiplier;
                    r *= slowModeMultiplier;
                }

                follower.setTeleOpDrive(
                        y,
                        x,
                        r,
                        true
                );
            }
        }

        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive(true);
            automatedDrive = false;
        }

        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        telemetryM.debug("vx", vx);
        telemetryM.debug("vy", vy);
        telemetryM.debug("automatedDrive", automatedDrive);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
