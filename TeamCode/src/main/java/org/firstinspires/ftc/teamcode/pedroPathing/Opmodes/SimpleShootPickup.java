package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@Autonomous(name = "Simple Shoot + Pickup")
public class SimpleShootPickup extends OpMode {

    private Follower follower;

    private DcMotorEx shooterR, shooterL, intake;
    private Servo leftIndex, rightIndex, flicker;
    private DcMotor turret;

    private final double[] shootPositions = {0.31, 0.4, 0.49};
    private final double[] intakePositions = {0.084, 0.174, 0.264};
    private final double flickerUp = 0.45;
    private final double flickerDown = 0.7;

    private static final double SHOOTER_VELOCITY = 1280;

    private Pose startPose = new Pose(13, 127, Math.toRadians(145));
    private Pose shootPose = new Pose(55, 84, Math.toRadians(180));

    private final Pose pickup1 = new Pose(23, 84, Math.toRadians(180));
    private final Pose pickup2 = new Pose(23, 60, Math.toRadians(180));
    private final Pose pickup3 = new Pose(23, 36, Math.toRadians(180));

    private PathChain pathToShoot;
    private PathChain pathToPickup1;
    private PathChain pathToPickup2;
    private PathChain pathToPickup3;

    private enum ShootState {
        START, FLICK1, SPINDEX1, FLICK2, SPINDEX2, FLICK3, DONE
    }

    private ShootState shootState = ShootState.START;
    private long shootTimer;

    private int pickupStep = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");
        flicker = hardwareMap.get(Servo.class, "flicker");

        turret = hardwareMap.get(DcMotor.class, "turret");

        setSpindex(0);
        flicker.setPosition(flickerDown);

        pathToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        pathToPickup1 = follower.pathBuilder().addPath(new BezierLine(shootPose, pickup1)).build();
        pathToPickup2 = follower.pathBuilder().addPath(new BezierLine(pickup1, pickup2)).build();
        pathToPickup3 = follower.pathBuilder().addPath(new BezierLine(pickup2, pickup3)).build();
    }

    @Override
    public void start() {
        follower.followPath(pathToShoot, true);
        shooterR.setVelocity(SHOOTER_VELOCITY);
        shooterL.setVelocity(SHOOTER_VELOCITY);
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        follower.update();

        // --- Shooting Preload ---
        switch (shootState) {
            case START:
                if (!follower.isBusy()) {
                    flicker.setPosition(flickerUp);
                    shootTimer = now + 200;
                    shootState = ShootState.FLICK1;
                }
                break;
            case FLICK1:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerDown);
                    shootTimer = now + 200;
                    shootState = ShootState.SPINDEX1;
                }
                break;
            case SPINDEX1:
                if (now >= shootTimer) {
                    setSpindex(1);
                    shootTimer = now + 300;
                    shootState = ShootState.FLICK2;
                }
                break;
            case FLICK2:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerUp);
                    shootTimer = now + 200;
                    shootState = ShootState.SPINDEX2;
                }
                break;
            case SPINDEX2:
                if (now >= shootTimer) {
                    setSpindex(2);
                    shootTimer = now + 300;
                    shootState = ShootState.FLICK3;
                }
                break;
            case FLICK3:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerDown);
                    shootState = ShootState.DONE;
                    intake.setPower(-1); // start intake immediately
                    follower.followPath(pathToPickup1, true);
                }
                break;
            case DONE:
                break;
        }

        // --- Pickup First 3 Balls ---
        if (shootState == ShootState.DONE && !follower.isBusy()) {
            pickupStep++;
            switch (pickupStep) {
                case 1:
                    setSpindexIntakePosition(1);
                    follower.followPath(pathToPickup2, true);
                    break;
                case 2:
                    setSpindexIntakePosition(2);
                    follower.followPath(pathToPickup3, true);
                    break;
                case 3:
                    intake.setPower(0); // stop intake
                    break;
            }
        }
    }

    private void setSpindex(int index) {
        leftIndex.setPosition(shootPositions[index]);
        rightIndex.setPosition(shootPositions[index]);
    }

    private void setSpindexIntakePosition(int index) {
        leftIndex.setPosition(intakePositions[index]);
        rightIndex.setPosition(intakePositions[index]);
    }

    @Override
    public void stop() {
        shooterR.setPower(0);
        shooterL.setPower(0);
        intake.setPower(0);
    }
}
