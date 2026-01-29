package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@TeleOp(name = "ShooterSequenceTeleop")
public class RapidFireTest extends OpMode {

    private DcMotorEx shooterR, intake, shooterL;
    private Servo leftIndex, rightIndex, flicker;

    private final double[] shootPositions = {0.31, 0.4, 0.49};
    private final double flickerUp = 0.4;
    private final double flickerDown = 0.7;

    private static final double SERVO_DEADZONE = 0.004;
    private double lastLeftIndexPos = -1;
    private double lastRightIndexPos = -1;

    private static final double TARGET_VELOCITY = 1400;
    private static final double VELOCITY_TOLERANCE = 20;

    // ===== TURRET + TRACKING =====
    private DcMotor turret;
    private Limelight3A limelight;
    private Follower follower;

    private final int TURRET_MAX = 510;
    private final int TURRET_MIN = -350;
    private final double MAX_POWER_GOAL = 0.6;
    private final double Kp_GOAL = 0.01;
    private final double goalX = 0;
    private final double goalY = 144;
    private int turretZero = 0;

    private enum ShooterState {
        IDLE,
        RAMPING,
        WAIT_VELOCITY,

        FLICK1_UP, FLICK1_DOWN,
        SPINDEX2_WAIT,
        FLICK2_UP, FLICK2_DOWN,
        SPINDEX3_WAIT,
        FLICK3_UP, FLICK3_DOWN,

        DONE
    }

    private ShooterState shooterState = ShooterState.IDLE;
    private long stateTimer = 0;
    private boolean lastA = false;

    @Override
    public void init() {
        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
//        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");

        shooterR.setDirection(DcMotor.Direction.REVERSE);

        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");
        flicker = hardwareMap.get(Servo.class, "flicker");
        intake = hardwareMap.get(DcMotorEx.class, "intake");



        applyServoDeadzone(shootPositions[0]);
        flicker.setPosition(flickerDown);

        // ===== TURRET INIT =====
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(68, 81, Math.PI / 2));

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretZero = turret.getCurrentPosition();
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        boolean a = gamepad1.a;

        if (a && !lastA && shooterState == ShooterState.IDLE) {
            applyServoDeadzone(shootPositions[0]);
            shooterState = ShooterState.RAMPING;
        }

        switch (shooterState) {

            case IDLE:
                break;

            case RAMPING:
                shooterR.setVelocity(TARGET_VELOCITY);
                intake.setPower(-1);x
                shooterState = ShooterState.WAIT_VELOCITY;
                intake.setPower(-1);
                break;

            case WAIT_VELOCITY:
                shooterR.setVelocity(TARGET_VELOCITY);

                if (Math.abs(shooterR.getVelocity() - TARGET_VELOCITY) < VELOCITY_TOLERANCE) {
                    flicker.setPosition(flickerUp);
                    stateTimer = now + 200;
                    shooterState = ShooterState.FLICK1_UP;
                }
                break;

            case FLICK1_UP:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerDown);
                    stateTimer = now + 200;
                    shooterState = ShooterState.FLICK1_DOWN;
                }
                break;

            case FLICK1_DOWN:
                if (now >= stateTimer) {
                    applyServoDeadzone(shootPositions[1]);
                    stateTimer = now + 500;
                    shooterState = ShooterState.SPINDEX2_WAIT;
                }
                break;

            case SPINDEX2_WAIT:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerUp);
                    stateTimer = now + 200;
                    shooterState = ShooterState.FLICK2_UP;
                }
                break;

            case FLICK2_UP:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerDown);
                    stateTimer = now + 200;
                    shooterState = ShooterState.FLICK2_DOWN;
                }
                break;

            case FLICK2_DOWN:
                if (now >= stateTimer) {
                    applyServoDeadzone(shootPositions[2]);
                    stateTimer = now + 500;
                    shooterState = ShooterState.SPINDEX3_WAIT;
                }
                break;

            case SPINDEX3_WAIT:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerUp);
                    stateTimer = now + 200;
                    shooterState = ShooterState.FLICK3_UP;
                }
                break;

            case FLICK3_UP:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerDown);
                    stateTimer = now + 200;
                    shooterState = ShooterState.FLICK3_DOWN;
                }
                break;

            case FLICK3_DOWN:
                if (now >= stateTimer) {
                    stateTimer = now + 900;
                    shooterR.setPower(0);
                    intake.setPower(0);
                    shooterState = ShooterState.DONE;
                }
                break;

            case DONE:
                break;
        }

        lastA = a;

        // ===== TURRET TRACKING LOOP (FROM TELEOP) =====
        follower.update();
        Pose robotPose = follower.getPoseTracker().getPose();

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

        telemetry.addData("State", shooterState);
        telemetry.addData("Shooter Velocity", shooterR.getVelocity());
        telemetry.addData("Turret Pos", turret.getCurrentPosition());
        telemetry.addData("Turret Power", turretPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterR.setPower(0);
        turret.setPower(0);
        limelight.stop();
    }

    private void applyServoDeadzone(double pos) {
        double left = clamp(pos, 0.0, 1.0);
        double right = left;

        if (Math.abs(left - lastLeftIndexPos) > SERVO_DEADZONE) {
            leftIndex.setPosition(left);
            lastLeftIndexPos = left;
        }

        if (Math.abs(right - lastRightIndexPos) > SERVO_DEADZONE) {
            rightIndex.setPosition(right);
            lastRightIndexPos = right;
        }
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
