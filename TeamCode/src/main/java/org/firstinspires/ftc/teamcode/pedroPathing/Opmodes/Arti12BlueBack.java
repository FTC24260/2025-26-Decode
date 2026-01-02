package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@Autonomous(name = "12 Blue Back", group = "Autonomous")
public class Arti12BlueBack extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Pose definitions (same coordinates as original)
    private final Pose startPose = new Pose(57, 9, Math.toRadians(90));
    private final Pose scorePose1 = new Pose(72, 24, Math.toRadians(119.05));
    private final Pose scorePose2 = new Pose(60, 84, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(24, 36, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(24, 60, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(24, 84, Math.toRadians(180));
    private final Pose parkPose = new Pose(33, 33, Math.toRadians(90));
    private final Pose gatePose = new Pose(17, 66, Math.toRadians(180));

    // Control poses for bezier curves
    private final Pose pickup2ControlPose = new Pose(52, 59);
    private final Pose scorePose1ControlPose = new Pose(60, 60);

    // PathChain definitions
    private PathChain grabPickup2;
    private PathChain gate;
    private PathChain scorePickup2;
    private PathChain grabPickup1;
    private PathChain scorePickup1;
    private PathChain grabPickup3;
    private PathChain scorePickup3;
    private PathChain park;

    public void preStart() {
        // Build all paths using the new format
        grabPickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                pickup2ControlPose,
                                pickup2Pose
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup2Pose.getHeading())
                .build();

        gate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                pickup2Pose,
                                gatePose
                        )
                )
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                gatePose,
                                scorePose1
                        )
                )
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose1.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose1,
                                pickup1Pose
                        )
                )
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                pickup1Pose,
                                scorePose2
                        )
                )
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose2.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose2,
                                pickup3Pose
                        )
                )
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                pickup3Pose,
                                scorePose2
                        )
                )

                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose2.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose2,
                                parkPose
                        )
                )
                .setLinearHeadingInterpolation(scorePose2.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(gate, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    setPathState(-1); // Stop execution
                }
                break;
        }
    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        preStart();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        publishTelemetry();
    }

    public void publishTelemetry() {
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Is Busy", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean up if needed
    }
}