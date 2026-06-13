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

@Autonomous(name = "6 Blue Rear")
public class Arti6BlueRear extends OpMode {

    private Follower follower;

    private DcMotorEx shooterR, shooterL, intake;
    private DcMotor turret;

    private Servo leftIndex, rightIndex, flicker;

    private final double[] intakePositions = {0.2933, 0.4050, 0.5250};
    private final double[] shootPositions = {0.2306, 0.3467, 0.4689};

    private final double flickerUp = 0.575;
    private final double flickerDown = 0.795;

    private static final double SHOOTER_VELOCITY = 1810;
    private static final int TURRET_TARGET = 190;
    private static final double Kp_TURRET = 0.01;
    private static final double MAX_TURRET_POWER = 0.6;

    private Pose startPose = new Pose(63, 9, Math.toRadians(180));
    private Pose p110 = new Pose(20, 9, Math.toRadians(180));
    private Pose p115 = new Pose(15, 9, Math.toRadians(180));
    private Pose p120 = new Pose(10, 9, Math.toRadians(180));

    private PathChain to110;
    private PathChain to115;
    private PathChain to120;
    private PathChain backToStart;
    private PathChain final110;

    private enum State {
        WAIT_PRELOAD_DELAY,
        SHOOT_PRELOAD,
        GO_110,
        GO_115,
        GO_120,
        RETURN_START,
        SHOOT_SECOND,
        END_110,
        DONE
    }

    private State state = State.WAIT_PRELOAD_DELAY;

    private long preloadTimer;

    private enum ShootState {
        IDLE,
        FLICK1_UP, FLICK1_DOWN, SP1_WAIT,
        FLICK2_UP, FLICK2_DOWN, SP2_WAIT,
        FLICK3_UP, FLICK3_DOWN,
        DONE
    }

    private ShootState shootState = ShootState.IDLE;
    private long shootTimer;
    private long stateTimer;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");
        flicker = hardwareMap.get(Servo.class, "flicker");

        setSpindex(0);
        flicker.setPosition(flickerDown);

        to110 = follower.pathBuilder().addPath(new BezierLine(startPose, p110))
                .setConstantHeadingInterpolation(startPose.getHeading()).build();

        to115 = follower.pathBuilder().addPath(new BezierLine(p110, p115))
                .setConstantHeadingInterpolation(startPose.getHeading()).build();

        to120 = follower.pathBuilder().addPath(new BezierLine(p115, p120))
                .setConstantHeadingInterpolation(startPose.getHeading()).build();

        backToStart = follower.pathBuilder().addPath(new BezierLine(p120, startPose))
                .setConstantHeadingInterpolation(startPose.getHeading()).build();

        final110 = follower.pathBuilder().addPath(new BezierLine(startPose, p110))
                .setConstantHeadingInterpolation(startPose.getHeading()).build();
    }

    @Override
    public void start() {

        shooterR.setVelocity(SHOOTER_VELOCITY);
        shooterL.setVelocity(SHOOTER_VELOCITY);
        intake.setPower(-1);

        preloadTimer = System.currentTimeMillis() + 6000; // ONLY for preload
    }

    @Override
    public void loop() {

        follower.update();

        int error = TURRET_TARGET - turret.getCurrentPosition();
        double power = Kp_TURRET * error;
        power = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, power));
        turret.setPower(power);

        long now = System.currentTimeMillis();

        switch (state) {

            case WAIT_PRELOAD_DELAY:
                if (now >= preloadTimer) {
                    startRapidFire();
                    state = State.SHOOT_PRELOAD;
                }
                break;

            case SHOOT_PRELOAD:
                if (updateShooting(now)) {
                    setSpindexIntakePosition(0);
                    follower.followPath(to110, true);
                    state = State.GO_110;
                }
                break;

            case GO_110:
                if (!follower.isBusy()) {
                    setSpindexIntakePosition(1);
                    follower.followPath(to115, true);
                    state = State.GO_115;
                }
                break;

            case GO_115:
                if (!follower.isBusy()) {
                    setSpindexIntakePosition(2);
                    follower.followPath(to120, true);
                    state = State.GO_120;
                }
                break;

            case GO_120:
                if (!follower.isBusy()) {
                    setSpindex(0);
                    follower.followPath(backToStart, true);
                    state = State.RETURN_START;
                }
                break;

            case RETURN_START:
                if (!follower.isBusy()) {
                    turret.setTargetPosition(TURRET_TARGET+1);
                    startRapidFire(); // NO 3 second delay here
                    stateTimer = System.currentTimeMillis() + 500;
                    state = State.SHOOT_SECOND;
                }
                break;

            case SHOOT_SECOND:
                if (updateShooting(now)) {
                    follower.followPath(final110, true);
                    state = State.END_110;
                }
                break;

            case END_110:
                if (!follower.isBusy()) {
                    state = State.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    private void startRapidFire() {
        shootState = ShootState.IDLE; // RESET FIX
        flicker.setPosition(flickerUp);
        shootTimer = System.currentTimeMillis() + 200;
        shootState = ShootState.FLICK1_UP;
    }

    private boolean updateShooting(long now) {

        switch (shootState) {

            case FLICK1_UP:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerDown);
                    shootTimer = now + 200;
                    shootState = ShootState.FLICK1_DOWN;
                }
                break;

            case FLICK1_DOWN:
                if (now >= shootTimer) {
                    setSpindex(1);
                    shootTimer = now + 2500;
                    shootState = ShootState.SP1_WAIT;
                }
                break;

            case SP1_WAIT:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerUp);
                    shootTimer = now + 200;
                    shootState = ShootState.FLICK2_UP;
                }
                break;

            case FLICK2_UP:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerDown);
                    shootTimer = now + 200;
                    shootState = ShootState.FLICK2_DOWN;
                }
                break;

            case FLICK2_DOWN:
                if (now >= shootTimer) {
                    setSpindex(2);
                    shootTimer = now + 2500;
                    shootState = ShootState.SP2_WAIT;
                }
                break;

            case SP2_WAIT:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerUp);
                    shootTimer = now + 400;
                    shootState = ShootState.FLICK3_UP;
                }
                break;

            case FLICK3_UP:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerDown);
                    shootTimer = now + 500;
                    shootState = ShootState.DONE;
                }
                break;

            case DONE:
                return true;
        }

        return false;
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
        turret.setPower(0);
    }
}
