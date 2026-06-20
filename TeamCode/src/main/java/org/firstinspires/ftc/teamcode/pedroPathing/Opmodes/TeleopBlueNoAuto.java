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

    private final double[] intakePositions = {0.2272, 0.3478, 0.4667};
    private final double[] shootPositions = {0.1706, 0.2872, 0.4111};

    private final double flickerUp = 0.333;
    private final double flickerDown = 0.575;

    private static final double SERVO_DEADZONE = 0.004;
    private static final long SENSOR_IGNORE_MS = 800;
    private static final double SHOOTER_VELOCITY = 1430;

    private static final long SPINDEXER_SETTLE_MS = 25;
    private static final long FLICKER_UP_MS = 200;
    private static final long FLICKER_DOWN_MS = 200;

    private Servo rgbLight;

    private double lastIndexPos = -1;
    private int currentIndex = 0;
    private long ignoreSensorUntil = 0;

    private boolean sensorReady = true;

    private static final int TURRET_MIN = -300;
    private static final int TURRET_MAX = 400;

    private static final double TURRET_MIN_POWER = 0.2;
    private static final double TURRET_MAX_POWER = 0.3;

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
        rgbLight = hardwareMap.get(Servo.class, "rgbIndicator");

        flicker.setPosition(flickerDown);
        setSpindexPositionForce(intakePositions[0]);

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotor.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(13, 127, Math.PI / 2));
        follower.update();

        currentIndex = 0;
        ballsToShoot = 0;
        shotIndex = 0;
        shooterState = ShooterState.IDLE;
        sensorReady = true;
        ignoreSensorUntil = 0;
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        long now = System.currentTimeMillis();

        String detectedColor = detectColor();
        boolean seesBall = !detectedColor.equals("unknown");

        shooterR.setVelocity(SHOOTER_VELOCITY);
        shooterL.setVelocity(SHOOTER_VELOCITY);

        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y / 1.5,
                -gamepad1.left_stick_x / 2,
                -gamepad1.right_stick_x / 1.5,
                true
        );

        if (gamepad1.left_trigger > 0.1) {
            intake.setPower(-1);
        } else if (gamepad1.right_trigger > 0.1) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        if (shooterState == ShooterState.IDLE &&
                now >= ignoreSensorUntil &&
                currentIndex < intakePositions.length) {

            if (!seesBall) {
                sensorReady = true;
            } else if (sensorReady) {
                currentIndex++;
                sensorReady = false;
                ignoreSensorUntil = now + SENSOR_IGNORE_MS;

                if (currentIndex < intakePositions.length) {
                    setSpindexIntakePosition(currentIndex);
                } else {
                    setSpindexPositionForce(shootPositions[0]);
                }
            }
        }

        boolean a = gamepad1.a;

        if (shooterState == ShooterState.IDLE && a && !lastA && currentIndex > 0) {
            ballsToShoot = Math.min(currentIndex, shootPositions.length);
            shotIndex = 0;
            shooterState = ShooterState.SET_POS;
        }

        lastA = a;

        switch (shooterState) {

            case IDLE:
                break;

            case SET_POS:
                setSpindexPositionForce(shootPositions[shotIndex]);
                stateTimer = now + SPINDEXER_SETTLE_MS;
                shooterState = ShooterState.WAIT_POS;
                break;

            case WAIT_POS:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerUp);
                    stateTimer = now + FLICKER_UP_MS;
                    shooterState = ShooterState.WAIT_UP;
                }
                break;

            case WAIT_UP:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerDown);
                    stateTimer = now + FLICKER_DOWN_MS;
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
                setSpindexPositionForce(intakePositions[0]);
                currentIndex = 0;
                ballsToShoot = 0;
                shotIndex = 0;
                ignoreSensorUntil = now + SENSOR_IGNORE_MS;
                sensorReady = false;
                shooterState = ShooterState.IDLE;
                break;
        }

        if (gamepad1.dpad_left) {
            turret.setPower(0.25);
        } else if (gamepad1.dpad_right) {
            turret.setPower(-0.25);
        } else {
            turret.setPower(0);
        }

        if (gamepad1.x) {
            flicker.setPosition(flickerDown);
        }

        if (gamepad1.y) {
            flicker.setPosition(flickerUp);
        }

        if (currentIndex == intakePositions.length) {
            rgbLight.setPosition(0.5);
        } else {
            rgbLight.setPosition(0.6);
        }

        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Shooter Velocity R", shooterR.getVelocity());
        telemetry.addData("Shooter Velocity L", shooterL.getVelocity());
        telemetry.addData("Indexed Balls", currentIndex);
        telemetry.addData("Balls To Shoot", ballsToShoot);
        telemetry.addData("Shot Index", shotIndex);
        telemetry.addData("Color Detection", detectedColor);
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("Sensor Ready", sensorReady);
        telemetry.addData("Spindexer Position", lastIndexPos);
        telemetry.addData("Flicker Position", flicker.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterR.setPower(0);
        shooterL.setPower(0);
        intake.setPower(0);
        turret.setPower(0);
    }

    private String detectColor() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        if (g > 1.25 * r && g > 1.5 * b && g > 8) return "green";

        int maxRB = Math.max(r, b);
        int minRB = Math.min(r, b);

        if (maxRB > 25 && minRB >= 0 * maxRB && g < maxRB) return "purple";

        return "unknown";
    }

    private void setSpindexIntakePosition(int index) {
        if (index < 0) {
            index = 0;
        }

        if (index >= intakePositions.length) {
            index = intakePositions.length - 1;
        }

        applyServoDeadzone(intakePositions[index]);
    }

    private void applyServoDeadzone(double pos) {
        if (Math.abs(pos - lastIndexPos) > SERVO_DEADZONE) {
            setSpindexPositionForce(pos);
        }
    }

    private void setSpindexPositionForce(double pos) {
        leftIndex.setPosition(pos);
        rightIndex.setPosition(pos);
        lastIndexPos = pos;
    }
}