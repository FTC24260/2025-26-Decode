package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
    private DcMotor leftFront, leftRear, rightFront, rightRear;

    private DcMotorEx shooterR;
    private DcMotorEx shooterL;
    private DcMotorEx intake;

    private Servo leftIndex, rightIndex, flicker;
    private ColorSensor colorSensor;

    private final double[] shootPositions = {0.31, 0.4, 0.49};
    private final double[] intakePositions = {0.084, 0.174, 0.264};
    private final String[] slots = {"unknown", "unknown", "unknown"};
    private int currentIndex = 0;

    private final double flickerUp = 0.4;
    private final double flickerDown = 0.7;

    private static final double SERVO_DEADZONE = 0.004;
    private double lastLeftIndexPos = -1;
    private double lastRightIndexPos = -1;

    private static final double VELOCITY_TOLERANCE = 20;

    private long ignoreSensorUntil = 0;
    private static final long SENSOR_IGNORE_MS = 800;

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

    private double[] distPoints = {40, 60, 80, 100};
    private double[] powerPoints = {1210, 1380, 1500, 1700};
    private static final double FALLBACK_SHOOTER_VELOCITY = 1700;

    @Override
    public void init() {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        intake = hardwareMap.get(DcMotorEx.class, "intake");

        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");
        flicker = hardwareMap.get(Servo.class, "flicker");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        applyServoDeadzone(intakePositions[0]);
        flicker.setPosition(flickerDown);

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(75, 20, Math.PI / 2));

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretZero = turret.getCurrentPosition();
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        double y = -gamepad2.left_stick_y;
        double x = gamepad2.left_stick_x;
        double rx = gamepad2.right_stick_x;

        double lf = y + x + rx;
        double lr = y - x + rx;
        double rf = y - x - rx;
        double rr = y + x - rx;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(lr),
                                Math.max(Math.abs(rf), Math.abs(rr)))));

        leftFront.setPower(lf / max);
        leftRear.setPower(lr / max);
        rightFront.setPower(rf / max);
        rightRear.setPower(rr / max);

        boolean intakePressed = gamepad1.left_trigger > 0.1;
        intake.setPower((intakePressed || shooterState != ShooterState.IDLE && shooterState != ShooterState.DONE) ? -1 : 0);

        String detectedColor = detectColor();
        if (!detectedColor.equals("unknown") && now >= ignoreSensorUntil && currentIndex < 3) {
            slots[currentIndex] = detectedColor;
            currentIndex++;
            setSpindexIntakePosition(currentIndex);
            ignoreSensorUntil = now + SENSOR_IGNORE_MS;
        }

        double limelightDistance = getLimelightDistance();

        double targetVelocity;
        if (gamepad1.a && limelightDistance < 0) {
            targetVelocity = FALLBACK_SHOOTER_VELOCITY;
        } else {
            targetVelocity = getShooterVelocityFromDistance();
        }

        boolean a = gamepad1.a;
        if (a && !lastA && shooterState == ShooterState.IDLE) {
            applyServoDeadzone(shootPositions[0]);
            shooterState = ShooterState.RAMPING;
        }

        switch (shooterState) {

            case IDLE:
                break;

            case RAMPING:
                shooterR.setVelocity(targetVelocity);
                shooterL.setVelocity(targetVelocity);
                shooterState = ShooterState.WAIT_VELOCITY;
                break;

            case WAIT_VELOCITY:
                shooterR.setVelocity(targetVelocity);
                shooterL.setVelocity(targetVelocity);

                if (Math.abs(shooterR.getVelocity() - targetVelocity) < VELOCITY_TOLERANCE) {
                    flicker.setPosition(flickerUp);
                    stateTimer = now + 170;
                    shooterState = ShooterState.FLICK1_UP;
                }
                break;

            case FLICK1_UP:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerDown);
                    stateTimer = now + 170;
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
                    stateTimer = now + 170;
                    shooterState = ShooterState.FLICK2_UP;
                }
                break;

            case FLICK2_UP:
                if (now >= stateTimer) {
                    flicker.setPosition(flickerDown);
                    stateTimer = now + 350;
                    shooterState = ShooterState.FLICK2_DOWN;
                }
                break;

            case FLICK2_DOWN:
                if (now >= stateTimer) {
                    applyServoDeadzone(shootPositions[2]);
                    stateTimer = now + 170;
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
                    shooterR.setPower(0);
                    shooterL.setPower(0);
                    intake.setPower(0);
                    resetToIntake();
                    shooterState = ShooterState.IDLE;
                }
                break;

            case DONE:
                break;
        }

        lastA = a;

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
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Distance", getLimelightDistance());
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

    private double getLimelightDistance() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;
        double ta = result.getTa();
        double k = 50.0;
        return k / Math.sqrt(ta); // just use whatever ta you get
    }

    private void resetToIntake() {
        currentIndex = 0;
        for (int i = 0; i < slots.length; i++) slots[i] = "unknown";
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

        if (d < distPoints[0]) return powerPoints[0];
        return powerPoints[powerPoints.length - 1];
    }
}
