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

@TeleOp(name = "TeleopWithTurretAuto")
public class TeleopNoAutoIntake extends OpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private TelemetryManager telemetryM;

    private DcMotor intake;
    private DcMotorEx shooterL, shooterR;

    private DcMotor turret;
    private Limelight3A limelight;

    public static double kV = 0.0005;
    public static double kS = 0.4;
    public static double targetVelocity = 100;

    private ColorSensor colorSensor;
    private Servo leftIndex, rightIndex, flicker;

    private final double[] intakePositions = {0.02, 0.11, 0.2};
    private final double[] shootPositions  = {0.25, 0.34, 0.43};
    private final String[] slots = {"unknown", "unknown", "unknown"};
    private int currentIndex = 0;

    private long ignoreSensorUntil = 0;
    private static final long SENSOR_IGNORE_MS = 250;

    private long initialIgnoreUntil = 0;
    private static final long INITIAL_IGNORE_MS = 400;

    private long postRapidIgnoreUntil = 0;
    private static final long POST_RAPID_IGNORE_MS = 800;

    private long intakeBurstUntil = 0;
    private static final long INTAKE_BURST_MS = 1000;

    private final double flickerUp = 0.45;
    private final double flickerDown = 0.7;

    private enum RapidFireState { IDLE, SPINUP_WAIT, FLICK_UP, RESET_WAIT }
    private RapidFireState rapidFireState = RapidFireState.IDLE;
    private int rapidFireIndex = 0;
    private long rapidFireTimer = 0;

    private boolean lastA = false;
    private boolean waitingForBallClear = false;

    private static final double SERVO_DEADZONE = 0.004;
    private double lastLeftIndexPos = -1;
    private double lastRightIndexPos = -1;

    // Turret auto-tracking constants
    private final double deadzone = 1;
    private final double kP = 0.03;
    private final double maxPower = 1;
    private final double minPower = 0.07;
    private final int maxPosition = 430;
    private final int minPosition = -500;

    // Keep track of last known direction (+1 = clockwise, -1 = counterclockwise)
    private int lastTurretDirection = 0;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Drive motors
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

        // Intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Shooter
        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);

        // Color sensor & spindex
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");
        flicker = hardwareMap.get(Servo.class, "flicker");
        setSpindexIntakePosition(0);
        flicker.setPosition(flickerDown);

        // Turret
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(REVERSE);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void start() {
        initialIgnoreUntil = System.currentTimeMillis() + INITIAL_IGNORE_MS;
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        // ---------------- Drive ----------------
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

        // ---------------- Intake ----------------
        boolean intakePressed = gamepad1.left_trigger > 0.1;
        intake.setPower((intakePressed || now < intakeBurstUntil) ? -1 : 0);

        String detectedColor = detectColor();

        if (detectedColor.equals("unknown")) {
            waitingForBallClear = false;
        }

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

        // ---------------- Shooter ----------------
        if (rapidFireState != RapidFireState.IDLE) {
            double power = clamp(feedforward(targetVelocity), 0, 1);
            shooterL.setPower(power);
            shooterR.setPower(power);
        }

        if (rapidFireState == RapidFireState.SPINUP_WAIT) {
            setSpindexShootPosition(rapidFireIndex);
            if (now >= rapidFireTimer) {
                flicker.setPosition(flickerUp);
                rapidFireTimer = now + 200; // FLICK_UP_MS
                rapidFireState = RapidFireState.FLICK_UP;
            }
        } else if (rapidFireState == RapidFireState.FLICK_UP) {
            if (now >= rapidFireTimer) {
                flicker.setPosition(flickerDown);
                slots[rapidFireIndex] = "unknown";

                if (rapidFireIndex < 2 && anySlotLoaded()) {
                    rapidFireIndex++;
                    intakeBurstUntil = now + INTAKE_BURST_MS;
                    rapidFireTimer = now + 1000; // SPINUP_DELAY_MS
                    rapidFireState = RapidFireState.SPINUP_WAIT;
                } else {
                    rapidFireTimer = now + 200; // RESET_DELAY_MS
                    rapidFireState = RapidFireState.RESET_WAIT;
                }
            }
        } else if (rapidFireState == RapidFireState.RESET_WAIT) {
            if (now >= rapidFireTimer) {
                shooterL.setPower(0);
                shooterR.setPower(0);
                rapidFireIndex = 0;
                currentIndex = 0;
                setSpindexIntakePosition(0);
                postRapidIgnoreUntil = now + POST_RAPID_IGNORE_MS;
                rapidFireState = RapidFireState.IDLE;
            }
        }

        if (gamepad1.a && !lastA && anySlotLoaded()) {
            intakeBurstUntil = now + INTAKE_BURST_MS;
            rapidFireIndex = 0;
            rapidFireTimer = now + 1000; // SPINUP_DELAY_MS
            rapidFireState = RapidFireState.SPINUP_WAIT;
        }
        lastA = gamepad1.a;

        // ---------------- Turret Auto-Tracking with prediction ----------------
        int currentPos = turret.getCurrentPosition();
        double turretPower = 0;
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double error = result.getTy();

            if (Math.abs(error) > deadzone) {
                turretPower = kP * error;

                if (turretPower > 0) turretPower = Math.max(turretPower, minPower);
                else turretPower = Math.min(turretPower, -minPower);

                turretPower = Math.max(-maxPower, Math.min(maxPower, turretPower));

                // Update last known direction
                lastTurretDirection = (turretPower > 0) ? 1 : -1;
            } else {
                turretPower = 0;
                lastTurretDirection = 0;
            }
        } else {
            // Target lost: keep turning in last known direction
            turretPower = minPower * lastTurretDirection;
        }

        // Respect physical bounds
        if ((currentPos >= maxPosition && turretPower > 0) ||
                (currentPos <= minPosition && turretPower < 0)) {
            turretPower = 0;
        }

        turret.setPower(turretPower);

        // ---------------- Telemetry ----------------
        telemetry.addData("Slots", slots[0] + ", " + slots[1] + ", " + slots[2]);
        telemetry.addData("RapidFire", rapidFireState);
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

    // ---------------- Helper Methods ----------------
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
        return !slots[0].equals("unknown")
                || !slots[1].equals("unknown")
                || !slots[2].equals("unknown");
    }

    private double feedforward(double targetVel) {
        return kS * Math.signum(targetVel) + kV * targetVel;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
