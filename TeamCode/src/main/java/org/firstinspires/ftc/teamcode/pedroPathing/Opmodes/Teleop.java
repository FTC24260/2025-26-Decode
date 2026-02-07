package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {

    private DcMotorEx shooterR, shooterL, intake;
    private Servo leftIndex, rightIndex, flicker;
    private ColorSensor colorSensor;

    private Follower follower;

    private DcMotor turret;
    private Limelight3A limelight;

    private final double[] shootPositions = {0.31, 0.4, 0.49};
    private final double[] intakePositions = {0.084, 0.174, 0.264};
    private int currentIndex = 0;

    private final double flickerUp = 0.54;
    private final double flickerDown = 0.76;

    private static final double SERVO_DEADZONE = 0.004;
    private double lastIndexPos = -1;

    private static final double VELOCITY_TOLERANCE = 20;
    private static final long SENSOR_IGNORE_MS = 800;
    private long ignoreSensorUntil = 0;

    private enum ShooterState {
        IDLE,
        RAMPING,
        WAIT_VELOCITY,
        WAIT_LONG_DIST,
        FLICK1_UP, FLICK1_DOWN,
        SPINDEX2_WAIT,
        FLICK2_UP, FLICK2_DOWN,
        SPINDEX3_WAIT,
        FLICK3_UP, FLICK3_DOWN
    }

    private ShooterState shooterState = ShooterState.IDLE;
    private long stateTimer = 0;
    private boolean lastA = false;

    private final int TURRET_MAX = 450;
    private final int TURRET_MIN = -460;
    private final double MAX_POWER_GOAL = 0.4;
    private final double goalX = 13;
    private final double goalY = 144;

    private double[] distPoints = {44, 50, 80, 100};
    private double[] powerPoints = {1250, 1320, 1500, 1950};
    private static final double FALLBACK_SHOOTER_VELOCITY = 1950;

    private double latchedTargetVelocity = 0;
    private boolean longDistanceFirstShot = false;

    @Override
    public void init() {
        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        intake   = hardwareMap.get(DcMotorEx.class, "intake");

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftIndex  = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");
        flicker    = hardwareMap.get(Servo.class, "flicker");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        flicker.setPosition(flickerDown);
        applyServoDeadzone(intakePositions[0]);

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(73, 63, Math.PI / 2));
        follower.update();

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        // --- Teleop drive ---
        follower.update();
        follower.setTeleOpDrive(
                -gamepad2.left_stick_y / 1.4,
                -gamepad2.left_stick_x / 2,
                -gamepad2.right_stick_x / 1.4,
                true
        );

        // --- Intake control ---
        boolean intakePressed = gamepad1.left_trigger > 0.1;
        intake.setPower((intakePressed || shooterState != ShooterState.IDLE) ? -1 : 0);

        // --- Color sensor indexing ---
        if (now >= ignoreSensorUntil && currentIndex < 3) {
            String color = detectColor();
            if (!color.equals("unknown")) {
                currentIndex++;
                setSpindexIntakePosition(currentIndex);
                ignoreSensorUntil = now + SENSOR_IGNORE_MS;
            }
        }

        // --- Shooter trigger ---
        boolean a = gamepad1.a;
        if (a && !lastA && shooterState == ShooterState.IDLE) {
            latchedTargetVelocity =
                    (getLimelightDistance() < 0)
                            ? FALLBACK_SHOOTER_VELOCITY
                            : getShooterVelocityFromDistance();

            applyServoDeadzone(shootPositions[0]);
            longDistanceFirstShot = getLimelightDistance() > 60;
            shooterState = ShooterState.RAMPING;
        }

        // --- Shooter state machine ---
        switch (shooterState) {

            case RAMPING:
                shooterR.setVelocity(latchedTargetVelocity);
                shooterL.setVelocity(latchedTargetVelocity);
                shooterState = ShooterState.WAIT_VELOCITY;
                break;

            case WAIT_VELOCITY:
                if (Math.abs(shooterR.getVelocity() - latchedTargetVelocity) < VELOCITY_TOLERANCE) {
                    if (longDistanceFirstShot) {
                        stateTimer = now + 200;
                        shooterState = ShooterState.WAIT_LONG_DIST;
                    } else {
                        flicker.setPosition(flickerUp);
                        stateTimer = now + 200;
                        shooterState = ShooterState.FLICK1_UP;
                    }
                }
                break;

            case WAIT_LONG_DIST:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerUp);
                    stateTimer = now + 200;
                    shooterState = ShooterState.FLICK1_UP;
                    longDistanceFirstShot = false;
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
                    stateTimer = now + 350;
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
                    stateTimer = now + 350;
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
                    intake.setPower(0);
                    shooterR.setPower(0);
                    shooterL.setPower(0);
                    latchedTargetVelocity = 0;
                    resetToIntake();
                    shooterState = ShooterState.IDLE;

                }
                break;
        }

        lastA = a;

        // --- Turret aiming using only localization ---
        Pose pose = follower.getPose();
        double angleToGoal = Math.atan2(goalY - pose.getY(), goalX - pose.getX()) - pose.getHeading();

// Wrap angle to [-PI, PI]
        while (angleToGoal > Math.PI) angleToGoal -= 2 * Math.PI;
        while (angleToGoal < -Math.PI) angleToGoal += 2 * Math.PI;

// Convert angle to encoder ticks
        double turretAngleToTicks = 1020; // tune this value (ticks per full 2π rotation)
        int targetTicks = (int) (angleToGoal / (2 * Math.PI) * turretAngleToTicks);

// Clamp target to physical limits
        if (targetTicks > TURRET_MAX) targetTicks = TURRET_MAX;
        if (targetTicks < TURRET_MIN) targetTicks = TURRET_MIN;

// Calculate shortest path error
        int currentTicks = turret.getCurrentPosition();
        int error = targetTicks - currentTicks;
        if (error > (TURRET_MAX - TURRET_MIN) / 2) error -= (TURRET_MAX - TURRET_MIN);
        if (error < -(TURRET_MAX - TURRET_MIN) / 2) error += (TURRET_MAX - TURRET_MIN);

// Apply proportional control
        double kP_turret = 0.01; // tune as needed
        double turretPower = kP_turret * error;

// Respect physical limits
        if ((currentTicks >= TURRET_MAX && turretPower > 0) || (currentTicks <= TURRET_MIN && turretPower < 0)) {
            turretPower = 0;
        }

        turret.setPower(Range.clip(turretPower, -MAX_POWER_GOAL, MAX_POWER_GOAL));



        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Shooter Vel", shooterR.getVelocity());
        telemetry.addData("Latched Vel", latchedTargetVelocity);
        telemetry.addData("Long Dist First Shot", longDistanceFirstShot);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterR.setPower(0);
        shooterL.setPower(0);
        turret.setPower(0);
        limelight.stop();
    }

    private String detectColor() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        if (g > 1.2 * r && g > 1.2 * b && g > 20) return "green";
        int maxRB = Math.max(r, b);
        int minRB = Math.min(r, b);
        if (maxRB > 40 && minRB >= 0.5 * maxRB && g < 0.7 * maxRB) return "purple";
        return "unknown";
    }

    private void setSpindexIntakePosition(int index) {
        if (index >= intakePositions.length) index = intakePositions.length - 1;
        applyServoDeadzone(intakePositions[index]);
    }

    private void applyServoDeadzone(double pos) {
        if (Math.abs(pos - lastIndexPos) > SERVO_DEADZONE) {
            leftIndex.setPosition(pos);
            rightIndex.setPosition(pos);
            lastIndexPos = pos;
        }
    }

    private double getLimelightDistance() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;
        return 50.0 / Math.sqrt(result.getTa());
    }

    private void resetToIntake() {
        currentIndex = 0;
        applyServoDeadzone(intakePositions[0]);
        ignoreSensorUntil = System.currentTimeMillis() + SENSOR_IGNORE_MS;
    }

    private double getShooterVelocityFromDistance() {
        double d = getLimelightDistance();
        if (d < 0) return powerPoints[0];

        for (int i = 0; i < distPoints.length - 1; i++) {
            if (d >= distPoints[i] && d <= distPoints[i + 1]) {
                double t = (d - distPoints[i]) / (distPoints[i + 1] - distPoints[i]);
                return powerPoints[i] + t * (powerPoints[i + 1] - powerPoints[i]);
            }
        }
        return powerPoints[powerPoints.length - 1];
    }
}
