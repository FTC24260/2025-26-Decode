package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@TeleOp(name = "TeleopWithTurretGoalTrack_PIDShooter")
public class TeleopNoAutoIntake extends OpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private TelemetryManager telemetryM;

    private DcMotor intake;
    private DcMotorEx shooterL, shooterR;

    private DcMotor turret;
    private Limelight3A limelight;

    private ColorSensor colorSensor;
    private Servo leftIndex, rightIndex, flicker;

    private final double[] shootPositions = {0.31, 0.4, 0.49};
    private final double[] intakePositions  = {0.084, 0.174, 0.264};
    private final String[] slots = {"unknown", "unknown", "unknown"};
    private int currentIndex = 0;

    private long ignoreSensorUntil = 0;
    private static final long SENSOR_IGNORE_MS = 800;

    private long initialIgnoreUntil = 0;
    private static final long INITIAL_IGNORE_MS = 400;

    private long postRapidIgnoreUntil = 0;
    private static final long POST_RAPID_IGNORE_MS = 800;

    private long intakeBurstUntil = 0;
    private static final long INTAKE_BURST_MS = 1500;

    private final double flickerUp = 0.4;
    private final double flickerDown = 0.7;

    private boolean lastA = false;
    private boolean waitingForBallClear = false;

    private static final double SERVO_DEADZONE = 0.004;
    private double lastLeftIndexPos = -1;
    private double lastRightIndexPos = -1;

    private Follower follower;
    private final int TURRET_MAX = 510;
    private final int TURRET_MIN = -350;
    private final double MAX_POWER_GOAL = 0.6;
    private final double Kp_GOAL = 0.01;
    private final double goalX = 0;
    private final double goalY = 144;
    private int turretZero = 0;

    private long lastLoopTime = 0;

    private enum ShooterState { IDLE, RAMPING, READY_TO_FLICK, FLICK_UP, FLICK_DOWN, COOLDOWN }
    private ShooterState shooterState = ShooterState.IDLE;
    private int shooterIndex = 0;
    private long stateTimer = 0;

    private static final long SHOOTER_DELAY_MS = 1300;
    private static final long SHOOTER_COOLDOWN_MS = 5000;

    private double[] shooterTargetPowerPerShot = {0.075, 0.069, 0.087}; // customize each shot power here

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");

        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");
        flicker = hardwareMap.get(Servo.class, "flicker");
        setSpindexIntakePosition(0);
        flicker.setPosition(flickerDown);

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(REVERSE);

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
    public void start() {
        initialIgnoreUntil = System.currentTimeMillis() + INITIAL_IGNORE_MS;
        lastLoopTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();
        lastLoopTime = now;

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
        intake.setPower((intakePressed || shooterState != ShooterState.IDLE && shooterState != ShooterState.COOLDOWN) ? -1 : 0);



        String detectedColor = detectColor();
        if (detectedColor.equals("unknown")) waitingForBallClear = false;

        if (!waitingForBallClear
                && !detectedColor.equals("unknown")
                && now >= ignoreSensorUntil
                && now >= initialIgnoreUntil
                && now >= postRapidIgnoreUntil
                && currentIndex < 3) {
            slots[currentIndex] = detectedColor;
            currentIndex++;
            setSpindexIntakePosition(currentIndex);
            ignoreSensorUntil = now + SENSOR_IGNORE_MS;
            waitingForBallClear = true;
        }

        // Inside your loop(), modify the shooter state handling like this:

        switch (shooterState) {
            case IDLE:
                if (gamepad1.a && !lastA && anySlotLoaded()) {
                    shooterIndex = 0;
                    intake.setPower(-1); // start intake burst for rapid fire
                    shooterState = ShooterState.RAMPING;
                }
                break;

            case RAMPING:
                double rampVelocity = shooterTargetPowerPerShot[shooterIndex] * 16000;
                shooterL.setVelocity(rampVelocity);
                shooterR.setVelocity(rampVelocity);
                setSpindexShootPosition(shooterIndex);
                stateTimer = now + SHOOTER_DELAY_MS;
                shooterState = ShooterState.READY_TO_FLICK;
                break;

            case READY_TO_FLICK:
                shooterL.setVelocity(shooterTargetPowerPerShot[shooterIndex] * 16000);
                shooterR.setVelocity(shooterTargetPowerPerShot[shooterIndex] * 16000);
                if (now >= stateTimer) {
                    flicker.setPosition(flickerUp);
                    stateTimer = now + 200;
                    shooterState = ShooterState.FLICK_UP;
                }
                break;

            case FLICK_UP:
                shooterL.setVelocity(shooterTargetPowerPerShot[shooterIndex] * 16000);
                shooterR.setVelocity(shooterTargetPowerPerShot[shooterIndex] * 16000);
                if (now >= stateTimer) {
                    flicker.setPosition(flickerDown);
                    stateTimer = now + 300;
                    shooterState = ShooterState.FLICK_DOWN;
                }
                break;

            case FLICK_DOWN:
                shooterL.setVelocity(shooterTargetPowerPerShot[shooterIndex] * 16000);
                shooterR.setVelocity(shooterTargetPowerPerShot[shooterIndex] * 16000);
                if (now >= stateTimer) {
                    shooterIndex++;
                    if (shooterIndex < 3 && anySlotLoaded()) {
                        setSpindexShootPosition(shooterIndex);
                        stateTimer = now + 400;
                        shooterState = ShooterState.READY_TO_FLICK;
                    } else {
                        stateTimer = now + SHOOTER_COOLDOWN_MS;
                        shooterState = ShooterState.COOLDOWN;
                    }
                }
                break;

            case COOLDOWN:
                shooterL.setVelocity(shooterTargetPowerPerShot[2] * 16000);
                shooterR.setVelocity(shooterTargetPowerPerShot[2] * 16000);
                intake.setPower(0); // stop intake after rapid fire
                if (now >= stateTimer) {
                    shooterL.setPower(0);
                    shooterR.setPower(0);
                    shooterState = ShooterState.IDLE;
                    currentIndex = 0;
                    setSpindexIntakePosition(0);
                    postRapidIgnoreUntil = now + POST_RAPID_IGNORE_MS;
                    intake.setPower(0);

                }
                break;
        }

        lastA = gamepad1.a;

        follower.update();
        Pose robotPose = follower.getPoseTracker().getPose();
        double dx = goalX - robotPose.getX();
        double dy = goalY - robotPose.getY();
        double targetAngle = Math.atan2(dy, dx) - robotPose.getHeading();

        double ticksPerRadian = (TURRET_MAX - TURRET_MIN) / (2 * Math.PI);
        int targetTicks = turretZero + (int)(targetAngle * ticksPerRadian);

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

        telemetry.addData("Slots", slots[0] + ", " + slots[1] + ", " + slots[2]);
        telemetry.addData("Shooter Power 1", shooterTargetPowerPerShot[0]);
        telemetry.addData("Shooter Power 2", shooterTargetPowerPerShot[1]);
        telemetry.addData("Shooter Power 3", shooterTargetPowerPerShot[2]);
        telemetry.addData("Shooter Velocity", shooterL.getVelocity());
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Turret Power", turretPower);
        telemetry.addData("Turret Pos", turret.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0);
        shooterL.setPower(0);
        shooterR.setPower(0);
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

    private void setSpindexShootPosition(int index) {
        if (index >= shootPositions.length) index = shootPositions.length - 1;
        applyServoDeadzone(shootPositions[index]);
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

    private boolean anySlotLoaded() {
        return !slots[0].equals("unknown") || !slots[1].equals("unknown") || !slots[2].equals("unknown");
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double getLimelightDistance() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;
        double ta = result.getTa();
        if (ta < 1.0) return -1;
        double k = 50.0;
        return k / Math.sqrt(ta);
    }

    private double linearInterpolation(double x) {
        return shooterTargetPowerPerShot[0]; // placeholder if needed, we override powers per shot
    }
}
