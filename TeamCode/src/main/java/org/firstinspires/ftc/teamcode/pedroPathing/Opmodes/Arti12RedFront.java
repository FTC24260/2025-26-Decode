package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@Autonomous(name = "12 Red Front")
public class Arti12RedFront extends OpMode {

    private Follower follower;

    private DcMotorEx shooterR, shooterL, intake;
    private Servo leftIndex, rightIndex, flicker;
    private DcMotor turret;

    private final double[] shootPositions = {0.29, 0.378, 0.472};
    private final double[] intakePositions = {0.144, 0.236, 0.33};
    private final double flickerUp = 0.575;
    private final double flickerDown = 0.795;

    private static final double SHOOTER_VELOCITY = 1360;

    private static final int TURRET_HOLD_POSITION = 140;
    private static final double Kp_TURRET = 0.01;
    private static final double MAX_TURRET_POWER = 0.5;

    private Pose startPose = new Pose(112, 122, Math.toRadians(37));
    private Pose shootPose = new Pose(83, 84, Math.toRadians(0));
    private final Pose finalPose = new Pose(111, 73, Math.toRadians(0));

    private final Pose pickup11Pose = new Pose(103, 84, Math.toRadians(0));
    private final Pose pickup12Pose = new Pose(109, 84, Math.toRadians(0));
    private final Pose pickup13Pose = new Pose(116, 84, Math.toRadians(0));
    private final Pose gatePose = new Pose(121, 74, Math.toRadians(0));


    private final Pose pickup21Pose = new Pose(102, 60, Math.toRadians(0));
    private final Pose pickup22Pose = new Pose(108, 60, Math.toRadians(0));
    private final Pose pickup23Pose = new Pose(115, 60, Math.toRadians(0));
    private final Pose pickup21Control = new Pose(85, 52);

    private final Pose pickup31Pose = new Pose(101, 36, Math.toRadians(0));
    private final Pose pickup32Pose = new Pose(108, 36, Math.toRadians(0));
    private final Pose pickup33Pose = new Pose(115, 36, Math.toRadians(0));
    private final Pose pickup31Control = new Pose(77, 48);

    private PathChain pathToShoot;
    private PathChain[] pickupPaths1;
    private PathChain[] pickupPaths2;
    private PathChain[] pickupPaths3;
    private PathChain returnToShootPath;
    private PathChain finalPath;

    private int cycle = 0;
    private int pickupState = 0;
    private boolean pickupStarted = false;
    private boolean returningToShoot = false;
    private boolean finalPathStarted = false;

    private boolean preloadDelayDone = false;
    private long preloadDelayEnd = 0;
    private long autoStartTime;
    private boolean waitAtShootPoseDone = false;
    private long shootPoseWaitEnd = 0;


    private enum ShootState {
        IDLE,
        FLICK1_UP, FLICK1_DOWN, SPINDEX1_WAIT,
        FLICK2_UP, FLICK2_DOWN, SPINDEX2_WAIT,
        FLICK3_UP, FLICK3_DOWN,
        DONE
    }

    private ShootState shootState = ShootState.IDLE;
    private long shootTimer;

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
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotor.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setSpindex(0);
        flicker.setPosition(flickerDown);

        pathToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        pickupPaths1 = new PathChain[]{
                follower.pathBuilder().addPath(new BezierLine(shootPose, pickup11Pose)).build(),
                follower.pathBuilder().addPath(new BezierLine(pickup11Pose, pickup12Pose)).build(),
                follower.pathBuilder().addPath(new BezierLine(pickup12Pose, pickup13Pose)).build()
        };

        pickupPaths2 = new PathChain[]{
                follower.pathBuilder().addPath(new BezierCurve(shootPose, pickup21Control, pickup21Pose))
                        .setConstantHeadingInterpolation(pickup21Pose.getHeading()).build(),
                follower.pathBuilder().addPath(new BezierLine(pickup21Pose, pickup22Pose)).build(),
                follower.pathBuilder().addPath(new BezierLine(pickup22Pose, pickup23Pose)).build()
        };

        pickupPaths3 = new PathChain[]{
                follower.pathBuilder().addPath(new BezierCurve(shootPose, pickup31Control, pickup31Pose))
                        .setConstantHeadingInterpolation(pickup31Pose.getHeading()).build(),
                follower.pathBuilder().addPath(new BezierLine(pickup31Pose, pickup32Pose)).build(),
                follower.pathBuilder().addPath(new BezierLine(pickup32Pose, pickup33Pose)).build()
        };
    }

    @Override
    public void start() {
        autoStartTime = System.currentTimeMillis();

        follower.followPath(pathToShoot, true);
        shooterR.setVelocity(SHOOTER_VELOCITY);
        shooterL.setVelocity(SHOOTER_VELOCITY);
        intake.setPower(-1);
    }


    @Override
    public void loop() {

        follower.update();

        // SIMPLE TURRET HOLD AT -150
        int error = TURRET_HOLD_POSITION - turret.getCurrentPosition();
        double power = Kp_TURRET * error;
        power = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, power));
        turret.setPower(power);

        long now = System.currentTimeMillis();

        if (cycle >= 3) {

            if (shootState != ShootState.DONE) {
                updateShooting(now);
                return;
            }

            if (!finalPathStarted && !follower.isBusy()) {

                finalPath = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), finalPose))
                        .setLinearHeadingInterpolation(
                                follower.getPose().getHeading(),
                                finalPose.getHeading())
                        .build();

                follower.followPath(finalPath, true);
                finalPathStarted = true;
            }

            return;
        }
        if (shootState == ShootState.IDLE && !follower.isBusy()) {
            // Only do this for the first cycle
            if (cycle == 0 && !waitAtShootPoseDone) {
//                shootPoseWaitEnd = now + 1500; // wait 1 second
                waitAtShootPoseDone = true;
                return; // keep looping until 1 second passes
            }

            // After the 1 second wait, start rapid fire
            if (cycle == 0 && waitAtShootPoseDone && now >= shootPoseWaitEnd) {
                flicker.setPosition(flickerUp);
                shootTimer = now + 200;
                shootState = ShootState.FLICK1_UP;
            }

            // For other cycles, start rapid fire immediately
            if (cycle != 0) {
                flicker.setPosition(flickerUp);
                shootTimer = now + 200;
                shootState = ShootState.FLICK1_UP;
            }
        }


        if (shootState != ShootState.DONE) {
            updateShooting(now);
            return;
        }

        PathChain[] active;
        if (cycle == 0) active = pickupPaths1;
        else if (cycle == 1) active = pickupPaths2;
        else active = pickupPaths3;

        if (!pickupStarted) {
            pickupStarted = true;
            pickupState = 0;
            setSpindexIntakePosition(0);
            follower.followPath(active[0], 0.6, true);
        }

        if (pickupStarted && pickupState < active.length && !follower.isBusy()) {
            pickupState++;
            if (pickupState < active.length) {
                setSpindexIntakePosition(pickupState);
                follower.followPath(active[pickupState], 0.6, true);
            } else {
                Pose last = (cycle == 0) ? pickup13Pose : (cycle == 1 ? pickup23Pose : pickup33Pose);

                if (cycle == 0) {
                    returnToShootPath = follower.pathBuilder()
                            .addPath(new BezierLine(pickup13Pose, shootPose))
                            .setLinearHeadingInterpolation(pickup13Pose.getHeading(), shootPose.getHeading() - Math.toRadians(3))
//                            .addPath(new BezierLine(gatePose, shootPose))
//                            .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose.getHeading() - Math.toRadians(3))
                            .build();
                } else {
                    returnToShootPath = follower.pathBuilder()
                            .addPath(new BezierLine(last, shootPose))
                            .setLinearHeadingInterpolation(last.getHeading(), shootPose.getHeading())
                            .build();
                }


                follower.followPath(returnToShootPath, true);
                returningToShoot = true;
                setSpindex(0);
            }
        }

        if (returningToShoot && !follower.isBusy()) {
            flicker.setPosition(flickerUp);
            shootTimer = now + 200;
            shootState = ShootState.FLICK1_UP;

            pickupStarted = false;
            returningToShoot = false;
            pickupState = 0;
            cycle++;
        }
    }

    private void updateShooting(long now) {
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
                    shootTimer = now + 500;
                    shootState = ShootState.SPINDEX1_WAIT;
                }
                break;

            case SPINDEX1_WAIT:
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
                    shootTimer = now + 500;
                    shootState = ShootState.SPINDEX2_WAIT;
                }
                break;

            case SPINDEX2_WAIT:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerUp);
                    shootTimer = now + 200;
                    shootState = ShootState.FLICK3_UP;
                }
                break;

            case FLICK3_UP:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerDown);
                    shootTimer = now + 500;
                    shootState = ShootState.FLICK3_DOWN;
                }
                break;

            case FLICK3_DOWN:
                if (now >= shootTimer) {
                    shootState = ShootState.DONE;
                }
                break;
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
        turret.setPower(0);
        intake.setPower(0);
    }
}
