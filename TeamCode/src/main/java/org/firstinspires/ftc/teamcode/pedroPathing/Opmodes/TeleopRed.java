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
public class TeleopRed extends OpMode {

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
    private final double goalX = 144;
    private final double goalY = 144;

    private double[] distPoints = {44, 50, 80, 100};
    private double[] powerPoints = {1300, 1320, 1500, 1900};
    private static final int TURRET_OFFSET_TICKS = 10; // tune experimentally

    private double latchedTargetVelocity = 0;

    private boolean manualOverride = false;
    private int turretManualOffset = 0;

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
        limelight.pipelineSwitch(1);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(95, 81, Math.PI / 2));
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
                -gamepad1.left_stick_y / 1.4,
                -gamepad1.left_stick_x / 2,
                -gamepad1.right_stick_x / 1.4,
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
        double limelightDist = getLimelightDistance();
        telemetry.addData("Limelight Distance", limelightDist);

        if (a && !lastA && shooterState == ShooterState.IDLE) {
            if (limelightDist > 0) {
                latchedTargetVelocity = getShooterVelocityFromDistance();
            } else {
                // Use localization distance if Limelight fails
                Pose pose = follower.getPose();
                double dx = goalX - pose.getX();
                double dy = goalY - pose.getY();
                double distance = Math.hypot(dx, dy);

                latchedTargetVelocity = powerPoints[0];
                for (int i = 0; i < distPoints.length - 1; i++) {
                    if (distance >= distPoints[i] && distance <= distPoints[i + 1]) {
                        double t = (distance - distPoints[i]) / (distPoints[i + 1] - distPoints[i]);
                        latchedTargetVelocity = powerPoints[i] + t * (powerPoints[i + 1] - powerPoints[i]);
                        break;
                    }
                }
            }

            applyServoDeadzone(shootPositions[0]);
            shooterState = ShooterState.RAMPING;
            stateTimer = System.currentTimeMillis() + 900;
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
                    if (System.currentTimeMillis() >= stateTimer) {
                        flicker.setPosition(flickerUp);
                        stateTimer = System.currentTimeMillis() + 200;
                        shooterState = ShooterState.FLICK1_UP;
                    }
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
                    latchedTargetVelocity = 0;
                    resetToIntake();
                    shooterState = ShooterState.IDLE;
                    stateTimer = now + 1000;
                    shooterR.setPower(0);
                    shooterL.setPower(0);
                }
                break;
        }

        lastA = a;

        // --- Manual turret override ---
        if (gamepad1.dpad_left) {
            turretManualOffset += 5;
            manualOverride = true;
        } else if (gamepad1.dpad_right) {
            turretManualOffset -= 5;
            manualOverride = true;
        }

        // --- Turret aiming ---
        Pose pose = follower.getPose();
        double angleToGoal = Math.atan2(goalY - pose.getY(), goalX - pose.getX()) - pose.getHeading();

        if (!manualOverride) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid() && Math.abs(result.getTy()) < 6) {
                angleToGoal += Math.toRadians(result.getTy());
            }
        }

        while (angleToGoal > Math.PI) angleToGoal -= 2 * Math.PI;
        while (angleToGoal < -Math.PI) angleToGoal += 2 * Math.PI;

        double turretAngleToTicks = 1020;
        int targetTicks = (int) (angleToGoal / (2 * Math.PI) * turretAngleToTicks) + TURRET_OFFSET_TICKS + turretManualOffset;
        targetTicks = Range.clip(targetTicks, TURRET_MIN, TURRET_MAX);

        int error = targetTicks - turret.getCurrentPosition();
        double kP_turret = 0.01;
        double turretPower = kP_turret * error;
        turret.setPower(Range.clip(turretPower, -MAX_POWER_GOAL, MAX_POWER_GOAL));

        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Shooter Vel", shooterR.getVelocity());
        telemetry.addData("Latched Vel", latchedTargetVelocity);
        telemetry.addData("Turret Offset", turretManualOffset);
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
