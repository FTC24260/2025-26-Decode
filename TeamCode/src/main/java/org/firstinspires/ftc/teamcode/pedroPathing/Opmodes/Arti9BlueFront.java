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

@Autonomous(name = "Arti9 Blue Front Auto - Drive Only", group = "Autonomous")
public class Arti9BlueFront extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // ==== Poses ====
    private final Pose startPose = new Pose(13, 127, Math.toRadians(145));
    private final Pose shootPreloadPose = new Pose(60, 84, Math.toRadians(180));

    private final Pose pickup11Pose = new Pose(38, 84, Math.toRadians(180));
    private final Pose pickup12Pose = new Pose(33, 84, Math.toRadians(180));
    private final Pose pickup13Pose = new Pose(28, 84, Math.toRadians(180));
    private final Pose shoot6Pose = new Pose(60, 84, Math.toRadians(180));

    private final Pose pickup21Control = new Pose(60, 64);
    private final Pose pickup21Pose = new Pose(38, 60, Math.toRadians(180));
    private final Pose pickup22Pose = new Pose(33, 60, Math.toRadians(180));
    private final Pose pickup23Pose = new Pose(28, 60, Math.toRadians(180));
    private final Pose shoot9Pose = new Pose(60, 84, Math.toRadians(180));

    private final Pose pickup31Control = new Pose(60, 34);
    private final Pose pickup31Pose = new Pose(38, 36, Math.toRadians(180));
    private final Pose pickup32Pose = new Pose(33, 36, Math.toRadians(180));
    private final Pose pickup33Pose = new Pose(28, 36, Math.toRadians(180));
    private final Pose shoot12Pose = new Pose(60, 84, Math.toRadians(180));

    // ==== Paths ====
    private PathChain shootPreload, pickup11, pickup12, pickup13, shoot6,
            pickup21, pickup22, pickup23, shoot9,
            pickup31, pickup32, pickup33, shoot12;

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        pathState = 0;
        // Start the first path immediately
        follower.followPath(shootPreload, true);
        pathState = 1;
    }


    @Override
    public void loop() {
        follower.update();  // Keep the follower moving
        autonomousPathUpdate();

        // Telemetry
        Pose p = follower.getPose();
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", p.getX());
        telemetry.addData("Y", p.getY());
        telemetry.addData("Heading", Math.toDegrees(p.getHeading()));
        telemetry.update();
    }

    // ====================== Path following ======================
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: if (!follower.isBusy()) { follower.followPath(shootPreload, true); pathState=1; } break;
            case 1: if (!follower.isBusy()) { follower.followPath(pickup11,true); pathState=2; } break;
            case 2: if (!follower.isBusy()) { follower.followPath(pickup12,true); pathState=3; } break;
            case 3: if (!follower.isBusy()) { follower.followPath(pickup13,true); pathState=4; } break;
            case 4: if (!follower.isBusy()) { follower.followPath(shoot6,true); pathState=5; } break;
            case 5: if (!follower.isBusy()) { follower.followPath(pickup21,true); pathState=6; } break;
            case 6: if (!follower.isBusy()) { follower.followPath(pickup22,true); pathState=7; } break;
            case 7: if (!follower.isBusy()) { follower.followPath(pickup23,true); pathState=8; } break;
            case 8: if (!follower.isBusy()) { follower.followPath(shoot9,true); pathState=9; } break;
            case 9: if (!follower.isBusy()) { follower.followPath(pickup31,true); pathState=10; } break;
            case 10: if (!follower.isBusy()) { follower.followPath(pickup32,true); pathState=11; } break;
            case 11: if (!follower.isBusy()) { follower.followPath(pickup33,true); pathState=12; } break;
            case 12: if (!follower.isBusy()) { follower.followPath(shoot12,true); pathState=13; } break;
        }
    }

    // ====================== Paths ======================
    private void buildPaths() {
        shootPreload = follower.pathBuilder().addPath(new BezierLine(startPose, shootPreloadPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPreloadPose.getHeading()).build();

        pickup11 = follower.pathBuilder().addPath(new BezierLine(shootPreloadPose, pickup11Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();

        pickup12 = follower.pathBuilder().addPath(new BezierLine(pickup11Pose, pickup12Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();

        pickup13 = follower.pathBuilder().addPath(new BezierLine(pickup12Pose, pickup13Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();

        shoot6 = follower.pathBuilder().addPath(new BezierLine(pickup13Pose, shoot6Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();

        pickup21 = follower.pathBuilder().addPath(new BezierCurve(shoot6Pose, pickup21Control, pickup21Pose))
                .setLinearHeadingInterpolation(shoot6Pose.getHeading(), pickup21Pose.getHeading()).build();

        pickup22 = follower.pathBuilder().addPath(new BezierLine(pickup21Pose, pickup22Pose))
                .setConstantHeadingInterpolation(pickup21Pose.getHeading()).build();

        pickup23 = follower.pathBuilder().addPath(new BezierLine(pickup22Pose, pickup23Pose))
                .setConstantHeadingInterpolation(pickup21Pose.getHeading()).build();

        shoot9 = follower.pathBuilder().addPath(new BezierLine(pickup23Pose, shoot9Pose))
                .setConstantHeadingInterpolation(pickup21Pose.getHeading()).build();

        pickup31 = follower.pathBuilder().addPath(new BezierCurve(shoot9Pose, pickup31Control, pickup31Pose))
                .setConstantHeadingInterpolation(pickup21Pose.getHeading()).build();

        pickup32 = follower.pathBuilder().addPath(new BezierLine(pickup31Pose, pickup32Pose))
                .setConstantHeadingInterpolation(pickup21Pose.getHeading()).build();

        pickup33 = follower.pathBuilder().addPath(new BezierLine(pickup32Pose, pickup33Pose))
                .setConstantHeadingInterpolation(pickup21Pose.getHeading()).build();

        shoot12 = follower.pathBuilder().addPath(new BezierLine(pickup33Pose, shoot12Pose))
                .setConstantHeadingInterpolation(pickup21Pose.getHeading()).build();
    }
}
