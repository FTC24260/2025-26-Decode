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

@Autonomous(name = "RapidFire_Auto_Test_Turret")
public class RapidFireAutoTest extends OpMode {

    private Follower follower;

    private DcMotorEx shooterR, shooterL;
    private Servo leftIndex, rightIndex, flicker;
    private DcMotor turret;

    private final double[] shootPositions = {0.31, 0.4, 0.49};
    private final double flickerUp = 0.4;
    private final double flickerDown = 0.7;

    private static final double VELOCITY_TOLERANCE = 20;
    private static final double TARGET_VELOCITY = 1440;

    // ===== Turret constants (same as teleop) =====
    private final int TURRET_MAX = 510;
    private final int TURRET_MIN = -350;
    private final double MAX_POWER_GOAL = 0.6;
    private final double Kp_GOAL = 0.01;
    private final double goalX = 12;
    private final double goalY = 144;
    private int turretZero = 0;

    private Pose startPose = new Pose(13, 127, Math.toRadians(145));
    private Pose shootPose = new Pose(60, 84, Math.toRadians(180));
    private PathChain pathToShoot;

    private int pathState = 0;

    private enum ShootState {
        IDLE,
        WAIT_VELOCITY,
        FLICK1_UP, FLICK1_DOWN, SPINDEX1_WAIT,
        FLICK2_UP, FLICK2_DOWN, SPINDEX2_WAIT,
        FLICK3_UP, FLICK3_DOWN,
        DONE
    }

    private ShootState shootState = ShootState.IDLE;
    private long shootTimer = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
    }

    @Override
    public void start() {
        follower.followPath(pathToShoot, true);
        pathState = 0;
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        follower.update();

        updateTurret(); // <<< turret tracking always running

        if (pathState == 0 && !follower.isBusy()) {
            startShooting();
            pathState = 1;
        }

        updateShooting(now);

        telemetry.addData("PathState", pathState);
        telemetry.addData("ShootState", shootState);
        telemetry.addData("TurretPos", turret.getCurrentPosition());
        telemetry.addData("Velocity", shooterR.getVelocity());
        telemetry.update();
    }

    private void startShooting() {
        shooterR.setVelocity(TARGET_VELOCITY);
        shooterL.setVelocity(TARGET_VELOCITY);
        shootState = ShootState.WAIT_VELOCITY;
    }

    private void updateShooting(long now) {
        switch (shootState) {

            case IDLE:
                break;

            case WAIT_VELOCITY:
                shooterR.setVelocity(TARGET_VELOCITY);
                shooterL.setVelocity(TARGET_VELOCITY);

                if (Math.abs(shooterR.getVelocity() - TARGET_VELOCITY) < VELOCITY_TOLERANCE) {
                    flicker.setPosition(flickerUp);
                    shootTimer = now + 200;
                    shootState = ShootState.FLICK1_UP;
                }
                break;

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
                    shootTimer = now + 200;
                    shootState = ShootState.FLICK3_DOWN;
                }
                break;

            case FLICK3_DOWN:
                if (now >= shootTimer) {
                    shooterR.setPower(0);
                    shooterL.setPower(0);
                    shootState = ShootState.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    // ===== Turret Tracking =====
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
        if (index >= shootPositions.length) index = shootPositions.length - 1;
        leftIndex.setPosition(shootPositions[index]);
        rightIndex.setPosition(shootPositions[index]);
    }

    @Override
    public void stop() {
        shooterR.setPower(0);
        shooterL.setPower(0);
        turret.setPower(0);
    }
}
