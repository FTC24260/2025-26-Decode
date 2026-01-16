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
    private final Pose startPose = new Pose(21, 121.8, Math.toRadians(143.9));
    private final Pose scorePose = new Pose(55, 75, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(10, 58, Math.toRadians(180));
    private final Pose gatePose = new Pose(12, 60, Math.toRadians(150));
    private final Pose pickup1Pose = new Pose(24, 84, Math.toRadians(90));

    // Control poses for bezier curves
    private final Pose pickup2Control = new Pose(44, 58);
    private final Pose shootControl = new Pose(20.25, 56.5);
    private final Pose pickup1Control = new Pose(52, 84);


    // PathChain definitions
    private PathChain shoot3;
    private PathChain intake2;
    private PathChain shoot6;
    private PathChain gateIntake1;
    private PathChain shoot9;
    private PathChain gateIntake2;
    private PathChain shoot12;
    private PathChain gateIntake3;
    private PathChain shoot15;
    private PathChain intake1;
    private PathChain shoot18;


    public void preStart() {
        // Build all paths using the new format
        shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        intake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                pickup2Pose
                        )
                )
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .build();

        shoot6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                pickup2Pose,
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        gateIntake1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                gatePose
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        shoot9 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                gatePose,
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        gateIntake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                gatePose
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        shoot12 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                gatePose,
                                scorePose
                        )
                )

                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        gateIntake3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                gatePose
                        )
                )
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        shoot15 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                gatePose,
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        intake1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                pickup1Pose
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        shoot18 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                pickup1Pose
                        )
                )
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(shoot3, true);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(intake2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(shoot6, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(gateIntake1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(shoot9, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(gateIntake2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(shoot12, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(gateIntake3, true);
                    setPathState(8);
                }
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(shoot15, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(intake1, true);
                    setPathState(10);
                }
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(shoot18, true);
                    setPathState(11);
                }
                break;
            case 11:
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