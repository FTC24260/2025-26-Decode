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

    private final double[] shootPositions = {0.31, 0.4, 0.49};
    private final double[] intakePositions = {0.084, 0.174, 0.264};
    private final double flickerUp = 0.45;
    private final double flickerDown = 0.7;

    private static final double PRELOAD_VEL_1 = 1230;
    private static final double PRELOAD_VEL_2 = 1450;
    private static final double PRELOAD_VEL_3 = 1330;
    private static final double SHOOTER_VELOCITY = 1320;

    private final int TURRET_MAX = 510;
    private final int TURRET_MIN = -350;
    private final double MAX_POWER_GOAL = 0.6;
    private final double Kp_GOAL = 0.01;
    private final double goalX = 130;
    private final double goalY = 144;
    private int turretZero;

    private Pose startPose = new Pose(131, 127, Math.toRadians(145));
    private Pose shootPose = new Pose(58, 84, Math.toRadians(180));

    private final Pose pickup11Pose = new Pose(36, 84, Math.toRadians(180));
    private final Pose pickup12Pose = new Pose(29, 84, Math.toRadians(180));
    private final Pose pickup13Pose = new Pose(24, 84, Math.toRadians(180));

    private final Pose pickup21Pose = new Pose(36, 60, Math.toRadians(180));
    private final Pose pickup22Pose = new Pose(29, 60, Math.toRadians(180));
    private final Pose pickup23Pose = new Pose(24, 60, Math.toRadians(180));
    private final Pose pickup21Control = new Pose(53, 52);

    private final Pose pickup31Pose = new Pose(36, 36, Math.toRadians(180));
    private final Pose pickup32Pose = new Pose(29, 36, Math.toRadians(180));
    private final Pose pickup33Pose = new Pose(24, 36, Math.toRadians(180));
    private final Pose pickup31Control = new Pose(48, 48);

    private final Pose gatePose = new Pose(20, 80, Math.toRadians(180));

    private PathChain pathToShoot;
    private PathChain[] pickupPaths1;
    private PathChain[] pickupPaths2;
    private PathChain[] pickupPaths3;
    private PathChain returnToShootPath;

    private int cycle = 0;
    private int pickupState = 0;
    private boolean pickupStarted = false;
    private boolean returningToShoot = false;

    private boolean preloadDelayDone = false;
    private long preloadDelayEnd = 0;

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
        turretZero = turret.getCurrentPosition();

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
        follower.followPath(pathToShoot, true);
        setShooterVelocity(PRELOAD_VEL_1);
        intake.setPower(-1);
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        follower.update();
        updateTurret();

        if (shootState == ShootState.IDLE && !follower.isBusy()) {
            if (!preloadDelayDone) {
                preloadDelayEnd = now + 500;
                preloadDelayDone = true;
                return;
            }
            if (now >= preloadDelayEnd) {
                setShooterVelocity(PRELOAD_VEL_1);
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
            follower.followPath(active[0], true);
        }

        if (pickupStarted && pickupState < active.length && !follower.isBusy()) {
            pickupState++;
            if (pickupState < active.length) {
                setSpindexIntakePosition(pickupState);
                follower.followPath(active[pickupState], true);
            } else {
                Pose last = (cycle == 0) ? pickup13Pose : (cycle == 1 ? pickup23Pose : pickup33Pose);

                if (cycle == 0) {
                    returnToShootPath = follower.pathBuilder()
                            .addPath(new BezierLine(pickup13Pose, gatePose))
                            .setConstantHeadingInterpolation(pickup13Pose.getHeading())
                            .addPath(new BezierLine(gatePose, shootPose))
                            .setLinearHeadingInterpolation(pickup13Pose.getHeading(), shootPose.getHeading())
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
            setShooterVelocity(SHOOTER_VELOCITY);
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
                    shootTimer = now + 800;
                    shootState = ShootState.SPINDEX1_WAIT;
                }
                break;

            case SPINDEX1_WAIT:
                if (now >= shootTimer) {
                    setShooterVelocity(PRELOAD_VEL_2);
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
                    shootTimer = now + 800;
                    shootState = ShootState.SPINDEX2_WAIT;
                }
                break;

            case SPINDEX2_WAIT:
                if (now >= shootTimer) {
                    setShooterVelocity(PRELOAD_VEL_3);
                    flicker.setPosition(flickerUp);
                    shootTimer = now + 200;
                    shootState = ShootState.FLICK3_UP;
                }
                break;

            case FLICK3_UP:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerDown);
                    shootTimer = now + 200;
                    shootState = ShootState.FLICK3_DOWN;
                }
                break;

            case FLICK3_DOWN:
                if (now >= shootTimer) {
                    setShooterVelocity(SHOOTER_VELOCITY);
                    shootState = ShootState.DONE;
                }
                break;
        }
    }

    private void updateTurret() {
        Pose robotPose = follower.getPose();

        double dx = goalX - robotPose.getX();
        double dy = goalY - robotPose.getY();
        double targetAngle = Math.atan2(dy, dx) - robotPose.getHeading();

        double ticksPerRadian = (TURRET_MAX - TURRET_MIN) / (2 * Math.PI);
        int targetTicks = turretZero + (int) (targetAngle * ticksPerRadian);

        int currentPos = turret.getCurrentPosition();
        int delta = targetTicks - currentPos;

        int maxRange = TURRET_MAX - TURRET_MIN;
        while (delta > maxRange / 2) delta -= maxRange;
        while (delta < -maxRange / 2) delta += maxRange;

        double turretPower = Kp_GOAL * delta;
        turretPower = Math.max(-MAX_POWER_GOAL, Math.min(MAX_POWER_GOAL, turretPower));

        if ((currentPos >= TURRET_MAX && turretPower > 0) ||
                (currentPos <= TURRET_MIN && turretPower < 0)) {
            turretPower = 0;
        }

        turret.setPower(turretPower);
    }

    private void setShooterVelocity(double vel) {
        shooterR.setVelocity(vel);
        shooterL.setVelocity(vel);
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
