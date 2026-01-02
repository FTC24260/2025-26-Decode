package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.RobotConstants;


@Autonomous(name = "Arti 18 Red Back", group = "Autonomous")
@Configurable // Panels
public class Arti18BlueBack extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain IntakePose2;
        public PathChain ScoreSmallTriangle6;
        public PathChain IntakePose1;
        public PathChain ScoreSmallTriangle9;
        public PathChain IntakePose3;
        public PathChain ScoreSmallTriangle12;
        public PathChain IntakeFromRamp;
        public PathChain ScoreSmallTriangle15;
        public PathChain ScoreSmallTriangle18;
        public PathChain AlmostGate;

        public Paths(Follower follower) {
            IntakePose2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.000, 9.000),
                                    new Pose(52.000, 62.000),
                                    new Pose(24.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            ScoreSmallTriangle6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 60.000), new Pose(72.000, 24.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(58.7)
                    )
                    .build();

            IntakePose1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 24.000), new Pose(24.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(58.7),
                            Math.toRadians(180)
                    )
                    .build();

            ScoreSmallTriangle9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 36.000), new Pose(72.000, 24.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(58.7)
                    )
                    .build();

            IntakePose3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(72.000, 24.000),
                                    new Pose(60.000, 80.000),
                                    new Pose(24.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(58.7),
                            Math.toRadians(170)
                    )
                    .build();

            ScoreSmallTriangle12 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 84.000), new Pose(72.000, 24.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(170),
                            Math.toRadians(58.7)
                    )
                    .build();

            IntakeFromRamp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 24.000), new Pose(13.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(58.7),
                            Math.toRadians(155)
                    )
                    .build();

            ScoreSmallTriangle15 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.000, 60.000), new Pose(72.000, 24.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(155),
                            Math.toRadians(58.7)
                    )
                    .build();

            IntakeFromRamp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 24.000), new Pose(13.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(58.7),
                            Math.toRadians(155)
                    )
                    .build();

            ScoreSmallTriangle18 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.000, 60.000), new Pose(72.000, 24.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(155),
                            Math.toRadians(58.7)
                    )
                    .build();

            AlmostGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 24.000), new Pose(19.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(58.7), Math.toRadians(0))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}