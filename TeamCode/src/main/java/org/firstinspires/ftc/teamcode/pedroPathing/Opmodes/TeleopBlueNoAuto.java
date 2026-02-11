package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@TeleOp(name = "TELEOP")
public class TeleopBlueNoAuto extends OpMode {

    private DcMotorEx shooterR, shooterL, intake;
    private DcMotor turret;
    private Servo leftIndex, rightIndex, flicker;
    private ColorSensor colorSensor;
    private Follower follower;

    private final double[] shootPositions = {0.31, 0.4, 0.49};
    private final double[] intakePositions = {0.084, 0.174, 0.264};

    private final double flickerUp = 0.575;
    private final double flickerDown = 0.795;

    private static final double SERVO_DEADZONE = 0.004;
    private static final long SENSOR_IGNORE_MS = 800;

    private double lastIndexPos = -1;
    private int currentIndex = 0;
    private long ignoreSensorUntil = 0;

    private static final double SHOOTER_LOW = 1400;
    private static final double SHOOTER_HIGH = 2000;

    private enum ShooterState {
        IDLE,
        RAMP_UP, WAIT_HIGH,
        BALL1_POS, BALL1_FLICK_UP, BALL1_FLICK_DOWN,
        BALL2_POS, BALL2_FLICK_UP, BALL2_FLICK_DOWN,
        BALL3_POS, BALL3_FLICK_UP, BALL3_FLICK_DOWN,
        WAIT_AFTER, RAMP_DOWN
    }

    private ShooterState shooterState = ShooterState.IDLE;
    private long stateTimer = 0;

    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void init() {
        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");
        flicker = hardwareMap.get(Servo.class, "flicker");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        flicker.setPosition(flickerDown);
        applyServoDeadzone(intakePositions[0]);

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotor.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(13, 127, Math.PI / 2));
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        shooterR.setVelocity(SHOOTER_LOW);
        shooterL.setVelocity(SHOOTER_LOW);
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        follower.update();
        follower.setTeleOpDrive(
                -gamepad2.left_stick_y / 1.4,
                -gamepad2.left_stick_x / 2,
                -gamepad2.right_stick_x / 1.4,
                true
        );

        intake.setPower(gamepad1.left_trigger > 0.1 ? -1 : 0);

        if (now >= ignoreSensorUntil && currentIndex < 3) {
            if (!detectColor().equals("unknown")) {
                currentIndex++;
                setSpindexIntakePosition(currentIndex);
                ignoreSensorUntil = now + SENSOR_IGNORE_MS;
            }
        }

        boolean a = gamepad1.a;
        boolean b = gamepad1.b;

        if (a && !lastA && shooterState == ShooterState.IDLE) {
            shooterState = ShooterState.BALL1_POS;
            stateTimer = now + 300;
        }

        if (b && !lastB && shooterState == ShooterState.IDLE) {
            shooterState = ShooterState.RAMP_UP;
        }

        switch (shooterState) {
            case IDLE:
                shooterR.setVelocity(SHOOTER_LOW);
                shooterL.setVelocity(SHOOTER_LOW);
                break;

            case RAMP_UP:
                shooterR.setVelocity(SHOOTER_HIGH);
                shooterL.setVelocity(SHOOTER_HIGH);
                stateTimer = now + 800;
                shooterState = ShooterState.WAIT_HIGH;
                break;

            case WAIT_HIGH:
                if (now >= stateTimer) {
                    shooterState = ShooterState.BALL1_POS;
                    stateTimer = now + 400;
                }
                break;

            case BALL1_POS:
                if (now >= stateTimer) {
                    applyServoDeadzone(shootPositions[0]);
                    stateTimer = now + 400;
                    shooterState = ShooterState.BALL1_FLICK_UP;
                }
                break;

            case BALL1_FLICK_UP:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerUp);
                    stateTimer = now + 180;
                    shooterState = ShooterState.BALL1_FLICK_DOWN;
                }
                break;

            case BALL1_FLICK_DOWN:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerDown);
                    stateTimer = now + 180;
                    shooterState = ShooterState.BALL2_POS;
                }
                break;

            case BALL2_POS:
                if (now >= stateTimer) {
                    applyServoDeadzone(shootPositions[1]);
                    stateTimer = now + 400;
                    shooterState = ShooterState.BALL2_FLICK_UP;
                }
                break;

            case BALL2_FLICK_UP:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerUp);
                    stateTimer = now + 180;
                    shooterState = ShooterState.BALL2_FLICK_DOWN;
                }
                break;

            case BALL2_FLICK_DOWN:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerDown);
                    stateTimer = now + 180;
                    shooterState = ShooterState.BALL3_POS;
                }
                break;

            case BALL3_POS:
                if (now >= stateTimer) {
                    applyServoDeadzone(shootPositions[2]);
                    stateTimer = now + 400;
                    shooterState = ShooterState.BALL3_FLICK_UP;
                }
                break;

            case BALL3_FLICK_UP:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerUp);
                    stateTimer = now + 400;
                    shooterState = ShooterState.BALL3_FLICK_DOWN;
                }
                break;

            case BALL3_FLICK_DOWN:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerDown);
                    stateTimer = now+1500;
                    shooterState = ShooterState.WAIT_AFTER;
                }
                break;

            case WAIT_AFTER:
                if (now >= stateTimer) {
                    resetToIntake();
                    shooterState = ShooterState.RAMP_DOWN;
                }
                break;

            case RAMP_DOWN:
                shooterR.setVelocity(SHOOTER_LOW);
                shooterL.setVelocity(SHOOTER_LOW);
                shooterState = ShooterState.IDLE;
                break;
        }

        lastA = a;
        lastB = b;

        double turretPower = 0;
        if (gamepad1.dpad_left) turretPower = 0.3;
        else if (gamepad1.dpad_right) turretPower = -0.3;
        turret.setPower(turretPower);

        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Shooter Vel", shooterR.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterR.setPower(0);
        shooterL.setPower(0);
        turret.setPower(0);
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

    private void resetToIntake() {
        currentIndex = 0;
        applyServoDeadzone(intakePositions[0]);
        ignoreSensorUntil = System.currentTimeMillis() + SENSOR_IGNORE_MS;
    }
}
