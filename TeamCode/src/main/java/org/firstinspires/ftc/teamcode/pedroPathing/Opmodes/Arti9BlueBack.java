package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@Autonomous(name = "9 Blue Back", group = "Autonomous")
@Configurable
public class Arti9BlueBack extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;

    private DcMotorEx shooterR, shooterL, intake;
    private Servo leftIndex, rightIndex, flicker;
    private DcMotor turret;

    private final double[] shootPositions = {0.31, 0.4, 0.49};
    private final double[] intakePositions = {0.084, 0.174, 0.264};
    private final double flickerUp = 0.54;
    private final double flickerDown = 0.76;


    private static final double SHOOTER_VELOCITY = 1340;

    private final int TURRET_MAX = 510;
    private final int TURRET_MIN = -350;
    private final double MAX_POWER_GOAL = 0.6;
    private final double Kp_GOAL = 0.01;
    private final double goalX = 15;
    private final double goalY = 144;
    private int turretZero;

    private int pathState = 0;

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
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

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

        shooterR.setVelocity(SHOOTER_VELOCITY);
        shooterL.setVelocity(SHOOTER_VELOCITY);
        intake.setPower(-1);

        paths = new Paths(follower);

        follower.followPath(paths.ShootPreload, true);
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
                flicker.setPosition(flickerUp);
                shootTimer = now + 200;
                shootState = ShootState.FLICK1_UP;
            }
        }

        if (shootState != ShootState.DONE) {
            updateShooting(now);
            return;
        }

        if (!follower.isBusy()) {
            switch (pathState) {
                case 0:
                    follower.followPath(paths.Pickup31, true);
                    setSpindexIntakePosition(0);
                    pathState++;
                    break;

                case 1:
                    follower.followPath(paths.Pickup32, true);
                    setSpindexIntakePosition(1);
                    pathState++;
                    break;

                case 2:
                    follower.followPath(paths.Pickup33, true);
                    setSpindexIntakePosition(2);
                    pathState++;
                    break;

                case 3:
                    follower.followPath(paths.Shoot3, true);
                    setSpindex(0);
                    shootState = ShootState.IDLE;
                    preloadDelayDone = false;
                    pathState++;
                    break;
            }
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

    public static class Paths {
        public PathChain ShootPreload, Pickup31, Pickup32, Pickup33, Shoot3;

        public Paths(Follower follower) {
            ShootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56, 8), new Pose(61, 22)))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Pickup31 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(61, 22), new Pose(36, 36)))
                    .build();

            Pickup32 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(36, 36), new Pose(29, 36)))
                    .build();

            Pickup33 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(29, 36), new Pose(24, 36)))
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(24, 36), new Pose(61, 10)))
                    .build();
        }
    }
}
