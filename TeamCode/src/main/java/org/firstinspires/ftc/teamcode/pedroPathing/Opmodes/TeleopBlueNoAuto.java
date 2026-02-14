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

    private final double[] shootPositions = {0.01, 0.104, 0.19};
    private final double[] intakePositions = {0.144, 0.236, 0.33};

    private final double flickerUp = 0.575;
    private final double flickerDown = 0.798;

    private static final double SERVO_DEADZONE = 0.004;
    private static final long SENSOR_IGNORE_MS = 800;
    private static final double SHOOTER_VELOCITY = 900;

    private double lastIndexPos = -1;
    private int currentIndex = 0;
    private long ignoreSensorUntil = 0;

    private enum ShooterState {
        IDLE,
        WAIT_SPINUP,
        BALL1_POS, BALL1_FLICK_UP, BALL1_FLICK_DOWN,
        BALL2_POS, BALL2_FLICK_UP, BALL2_FLICK_DOWN,
        BALL3_POS, BALL3_FLICK_UP, BALL3_FLICK_DOWN,
        WAIT_AFTER
    }

    private ShooterState shooterState = ShooterState.IDLE;
    private long stateTimer = 0;
    private boolean lastA = false;

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
    }

    @Override
    public void loop() {

        long now = System.currentTimeMillis();

        shooterR.setVelocity(SHOOTER_VELOCITY);
        shooterL.setVelocity(SHOOTER_VELOCITY);

        follower.update();
        follower.setTeleOpDrive(
                -gamepad2.left_stick_y / 1.4,
                -gamepad2.left_stick_x / 2,
                -gamepad2.right_stick_x / 1.4,
                true
        );

        if (gamepad1.left_trigger > 0.1) {
            intake.setPower(-1);
        }
        else if (gamepad1.right_trigger > 0.1) {
            intake.setPower(1);
        }
        else {
            intake.setPower(0);
        }
        if (shooterState == ShooterState.IDLE &&
                now >= ignoreSensorUntil &&
                currentIndex < 3 &&
                !detectColor().equals("unknown")) {

            currentIndex++;
            ignoreSensorUntil = now + SENSOR_IGNORE_MS;

            if (currentIndex < 3) {
                setSpindexIntakePosition(currentIndex);
            } else {
                applyServoDeadzone(shootPositions[0]);
            }
        }

        boolean a = gamepad1.a;

        if (shooterState == ShooterState.IDLE && currentIndex > 0) {
            if (a && !lastA) {
                stateTimer = now + 300;
                shooterState = ShooterState.WAIT_SPINUP;
            }
        }

        switch (shooterState) {

            case IDLE:
                break;

            case WAIT_SPINUP:
                if (now >= stateTimer) {
                    shooterState = ShooterState.BALL1_POS;
                    stateTimer = now + 200;
                }
                break;

            case BALL1_POS:
                if (now >= stateTimer) {
                    applyServoDeadzone(shootPositions[0]);
                    intake.setPower(-1);
                    stateTimer = now + 400;
                    shooterState = ShooterState.BALL1_FLICK_UP;
                }
                break;

            case BALL1_FLICK_UP:
                flicker.setPosition(flickerUp);
                stateTimer = now + 200;
                shooterState = ShooterState.BALL1_FLICK_DOWN;
                break;

            case BALL1_FLICK_DOWN:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerDown);

                    if (currentIndex >= 2) {
                        stateTimer = now + 400;
                        shooterState = ShooterState.BALL2_POS;
                    } else {
                        stateTimer = now + 400;
                        shooterState = ShooterState.WAIT_AFTER;
                    }
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
                flicker.setPosition(flickerUp);
                stateTimer = now + 200;
                shooterState = ShooterState.BALL2_FLICK_DOWN;
                break;

            case BALL2_FLICK_DOWN:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerDown);

                    if (currentIndex >= 3) {
                        stateTimer = now + 400;
                        shooterState = ShooterState.BALL3_POS;
                    } else {
                        stateTimer = now + 400;
                        shooterState = ShooterState.WAIT_AFTER;
                    }
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
                flicker.setPosition(flickerUp);
                stateTimer = now + 200;
                shooterState = ShooterState.BALL3_FLICK_DOWN;
                break;

            case BALL3_FLICK_DOWN:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerDown);
                    stateTimer = now + 400;
                    shooterState = ShooterState.WAIT_AFTER;
                }
                break;

            case WAIT_AFTER:
                if (now >= stateTimer) {
                    intake.setPower(0);
                    resetToIntake();
                    shooterState = ShooterState.IDLE;
                }
                break;
        }

        lastA = a;

        double turretPower = 0;
        if (gamepad1.dpad_left) turretPower = 0.3;
        else if (gamepad1.dpad_right) turretPower = -0.3;
        turret.setPower(turretPower);

        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Shooter Vel", shooterR.getVelocity());
        telemetry.addData("Indexed Balls", currentIndex);
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
