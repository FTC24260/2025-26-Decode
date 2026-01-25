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

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
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
        follower.setStartingPose(new Pose(21, 121.8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain ShootPreload3;
        public PathChain Pickup2;
        public PathChain Shoot6;
        public PathChain PickupGate1;
        public PathChain Shoot9;
        public PathChain PickupGate2;
        public PathChain Shoot12;
        public PathChain PickupGate3;
        public PathChain Shoot15;
        public PathChain Pickup1;
        public PathChain SHoot18;

        public Paths(Follower follower) {
            ShootPreload3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.000, 121.800), new Pose(55.000, 75.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(143.9),
                            Math.toRadians(180)
                    )
                    .build();

            Pickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(55.000, 75.000),
                                    new Pose(44.000, 58.000),
                                    new Pose(10.000, 58.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Shoot6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(10.000, 58.000),
                                    new Pose(20.250, 56.500),
                                    new Pose(55.000, 75.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            PickupGate1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.000, 75.000), new Pose(12.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                    .build();

            Shoot9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(12.000, 60.000), new Pose(55.000, 75.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(150))
                    .build();

            PickupGate2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.000, 75.000), new Pose(12.000, 60.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(150))
                    .build();

            Shoot12 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(12.000, 60.000), new Pose(55.000, 75.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(150))
                    .build();

            PickupGate3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.000, 75.000), new Pose(12.000, 60.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(150))
                    .build();

            Shoot15 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(12.000, 60.000), new Pose(55.000, 75.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))
                    .build();

            Pickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(55.000, 75.000),
                                    new Pose(52.000, 84.000),
                                    new Pose(24.000, 84.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            SHoot18 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 84.000), new Pose(55.000, 75.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootPreload3, true);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Pickup2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot6, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickupGate1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot9, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickupGate2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot12, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickupGate3, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot15, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Pickup1, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SHoot18, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    setPathState(-1); // Finished all paths
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
    }

}