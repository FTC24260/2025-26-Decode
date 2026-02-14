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

    private final double[] shootPositions = {0.28, 0.378, 0.467};
    private final double[] intakePositions = {0.144, 0.236, 0.33};

    private final double flickerUp = 0.575;
    private final double flickerDown = 0.798;

    private static final double SERVO_DEADZONE = 0.004;
    private static final long SENSOR_IGNORE_MS = 800;
    private static final double SHOOTER_VELOCITY = 1300;

    private double lastIndexPos = -1;
    private int currentIndex = 0;
    private long ignoreSensorUntil = 0;
    private static final int TURRET_MIN = -300;
    private static final int TURRET_MAX = 400;

    private static final double TURRET_MIN_POWER = 0.3;
    private static final double TURRET_MAX_POWER = 0.4;


    private enum ShooterState {
        IDLE,
        SET_POS,
        WAIT_POS,
        WAIT_UP,
        WAIT_DOWN,
        DONE
    }

    private ShooterState shooterState = ShooterState.IDLE;
    private long stateTimer = 0;

    private int ballsToShoot = 0;
    private int shotIndex = 0;

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

        if (gamepad1.left_trigger > 0.1) intake.setPower(-1);
        else if (gamepad1.right_trigger > 0.1) intake.setPower(1);
        else intake.setPower(0);

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

        if (shooterState == ShooterState.IDLE && a && !lastA && currentIndex > 0) {
            ballsToShoot = currentIndex;
            shotIndex = 0;
            shooterState = ShooterState.SET_POS;
        }

        lastA = a;

        switch (shooterState) {

            case IDLE:
                break;

            case SET_POS:

                if (!(ballsToShoot == 3 && shotIndex == 0)) {
                    applyServoDeadzone(shootPositions[shotIndex]);
                }

                stateTimer = now + 400;
                shooterState = ShooterState.WAIT_POS;
                break;

            case WAIT_POS:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerUp);
                    stateTimer = now + 200;
                    shooterState = ShooterState.WAIT_UP;
                }
                break;

            case WAIT_UP:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerDown);
                    stateTimer = now + 200;
                    shooterState = ShooterState.WAIT_DOWN;
                }
                break;

            case WAIT_DOWN:
                if (now >= stateTimer) {

                    shotIndex++;

                    if (shotIndex < ballsToShoot) {
                        shooterState = ShooterState.SET_POS;
                    } else {
                        shooterState = ShooterState.DONE;
                    }
                }
                break;

            case DONE:
                applyServoDeadzone(intakePositions[0]);
                currentIndex = 0;
                ignoreSensorUntil = now + SENSOR_IGNORE_MS;
                shooterState = ShooterState.IDLE;
                break;
        }

        double stick = gamepad1.left_stick_x;
        int turretPos = turret.getCurrentPosition();
        double turretPower = 0;

        if (Math.abs(stick) > 0.05) {

            double scaledPower = TURRET_MIN_POWER +
                    (TURRET_MAX_POWER - TURRET_MIN_POWER) * Math.abs(stick);

            if (stick > 0) {  // Turning positive direction

                if (turretPos >= TURRET_MAX) {
                    turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turret.setTargetPosition(TURRET_MIN);
                    turretPos = TURRET_MIN;
                } else {
                    turretPower = scaledPower;
                }

            } else {  // Turning negative direction

                if (turretPos <= TURRET_MIN) {
                    turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turret.setTargetPosition(TURRET_MAX);
                    turretPos = TURRET_MAX;
                } else {
                    turretPower = -scaledPower;
                }
            }
        }

        turret.setPower(turretPower);


        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Shooter Velocity", shooterR.getVelocity());
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
}
