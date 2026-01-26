package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@Autonomous(name = "Test First Movement", group = "Autonomous")
public class FirstMovementTest extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Pose definitions
    private final Pose startPose = new Pose(21, 121.8, Math.toRadians(143.9));
    private final Pose pickup2Pose = new Pose(30, 110, Math.toRadians(143.9));
    private final Pose pickup2ControlPose = new Pose(52, 59);

    // PathChain
    private PathChain grabPickup2;

    public void preStart() {
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup2Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup2Pose.getHeading())
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
                    setPathState(-1); // Stop execution when done
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
        // Optional: stop motors if needed
    }
}
