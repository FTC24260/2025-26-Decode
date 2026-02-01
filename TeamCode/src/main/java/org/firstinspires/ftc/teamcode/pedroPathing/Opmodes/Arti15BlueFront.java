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

@Autonomous(name = "15 Blue Front")
public class Arti15BlueFront extends OpMode {

    private Follower follower;

    private DcMotorEx shooterR, shooterL, intake;
    private Servo leftIndex, rightIndex, flicker;
    private DcMotor turret;

    private final double[] intakePositions = {0.31, 0.4, 0.49};
    private final double[] shootPositions = {0.084, 0.174, 0.264};
    private final double flickerUp = 0.45;
    private final double flickerDown = 0.7;

    private static final double SHOOTER_VELOCITY = 1340;

    private final int TURRET_MAX = 510;
    private final int TURRET_MIN = -350;
    private final double MAX_POWER_GOAL = 0.6;
    private final double Kp_GOAL = 0.01;
    private final double goalX = 17;
    private final double goalY = 144;
    private int turretZero;

    private Pose startPose = new Pose(13, 127, Math.toRadians(145));
    private Pose shootPose = new Pose(60, 84, Math.toRadians(180));

    private final Pose pickup1Ready = new Pose(40, 84, Math.toRadians(180));
    private final Pose pickup1 = new Pose(23, 84, Math.toRadians(180));

    private final Pose pickup2Ready = new Pose(40, 60, Math.toRadians(180));
    private final Pose pickup2 = new Pose(23, 60, Math.toRadians(180));
    private final Pose pickup2Control = new Pose(53, 52);

    private final Pose pickup3Ready = new Pose(40, 36, Math.toRadians(180));
    private final Pose pickup3 = new Pose(23, 36, Math.toRadians(180));
    private final Pose pickup3Control = new Pose(48, 48);

    private PathChain pathToShoot;
    private PathChain[][] pickupCycles;
    private PathChain returnToShootPath;

    private int cycle = 0;
    private int pickupState = 0;
    private boolean pickupStarted = false;
    private boolean returningToShoot = false;

    private enum IntakeSpindexState {
        TO_SLOT_0,
        WAIT_FOR_36,
        WAIT_70MS,
        DONE
    }

    private IntakeSpindexState intakeSpindexState = IntakeSpindexState.DONE;
    private long spindexTimer = 0;

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

        pickupCycles = new PathChain[][]{
                {
                        follower.pathBuilder().addPath(new BezierLine(shootPose, pickup1Ready)).build(),
                        follower.pathBuilder().addPath(new BezierLine(pickup1Ready, pickup1)).build()
                },
                {
                        follower.pathBuilder().addPath(new BezierCurve(shootPose, pickup2Control, pickup2Ready))
                                .setConstantHeadingInterpolation(pickup2Ready.getHeading()).build(),
                        follower.pathBuilder().addPath(new BezierLine(pickup2Ready, pickup2)).build()
                },
                {
                        follower.pathBuilder().addPath(new BezierCurve(shootPose, pickup3Control, pickup3Ready))
                                .setConstantHeadingInterpolation(pickup3Ready.getHeading()).build(),
                        follower.pathBuilder().addPath(new BezierLine(pickup3Ready, pickup3)).build()
                }
        };
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
        updateTurret();

        if (shootState == ShootState.IDLE && !follower.isBusy()) {
            flicker.setPosition(flickerUp);
            shootTimer = now + 200;
            shootState = ShootState.FLICK1_UP;
        }

        if (shootState != ShootState.DONE) {
            updateShooting(now);
            return;
        }

        PathChain[] active = pickupCycles[cycle];

        if (!pickupStarted) {
            pickupStarted = true;
            pickupState = 0;

            intakeSpindexState = IntakeSpindexState.TO_SLOT_0;
            setSpindexIntakePosition(0);

            intake.setPower(0);
            follower.followPath(active[0], true);
        }

        switch (intakeSpindexState) {
            case TO_SLOT_0:
                intakeSpindexState = IntakeSpindexState.WAIT_FOR_36;
                break;

            case WAIT_FOR_36:
                if (follower.getPose().getX() < 35.5) {
                    setSpindexIntakePosition(1);
                    spindexTimer = now + 200;
                    intakeSpindexState = IntakeSpindexState.WAIT_70MS;
                }
                break;

            case WAIT_70MS:
                if (now >= spindexTimer) {
                    setSpindexIntakePosition(2);
                    intakeSpindexState = IntakeSpindexState.DONE;
                }
                break;

            case DONE:
                break;
        }

        if (pickupStarted && !follower.isBusy()) {
            pickupState++;

            if (pickupState == 1) {
                intake.setPower(-1);
                follower.followPath(active[1], true);
            } else {
                intake.setPower(0);
                Pose last = cycle == 0 ? pickup1 : cycle == 1 ? pickup2 : pickup3;

                returnToShootPath = follower.pathBuilder()
                        .addPath(new BezierLine(last, shootPose))
                        .setLinearHeadingInterpolation(last.getHeading(), shootPose.getHeading())
                        .build();

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
                    shootTimer = now + 400;
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
                    shootTimer = now + 400;
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
                    shootTimer = now + 200;
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
