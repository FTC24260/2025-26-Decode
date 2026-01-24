package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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

@TeleOp(name = "TeleopWithTurretGoalTrack")
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

    private final double[] RAPID_FIRE_MAX_POWERS = {
            0.29,
            0.43,
            0.49
    };

    private long ignoreSensorUntil = 0;
    private static final long SENSOR_IGNORE_MS = 800;

    private long initialIgnoreUntil = 0;
    private static final long INITIAL_IGNORE_MS = 400;

    private long postRapidIgnoreUntil = 0;
    private static final long POST_RAPID_IGNORE_MS = 800;

    private long intakeBurstUntil = 0;
    private static final long INTAKE_BURST_MS = 1500;

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

    private Follower follower;
    private final int TURRET_MAX = 510;
    private final int TURRET_MIN = -350;
    private final double MAX_POWER_GOAL = 0.6;
    private final double Kp_GOAL = 0.021;
    private final double goalX = 0;
    private final double goalY = 144;
    private int turretZero = 0;

    public static final double[][] POWER_TABLE = {
            {12, 0.8},
            {24, 0.57},
            {36, 0.66},
            {48, 0.73},
            {60, 0.79}
    };

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
        intake.setPower((intakePressed || now < intakeBurstUntil) ? -1 : 0);

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

        if (rapidFireState == RapidFireState.IDLE && anySlotLoaded()) {
            if (gamepad1.a && !lastA) {
                rapidFireIndex = 0;
                rapidFireTimer = now + 900;
                rapidFireState = RapidFireState.SPINUP_WAIT;
            }
        }
        lastA = gamepad1.a;

        if (rapidFireState != RapidFireState.IDLE) {
            double power = clamp(feedforward(100), 0, RAPID_FIRE_MAX_POWERS[rapidFireIndex]);
            shooterL.setPower(power);
            shooterR.setPower(power);

            if (rapidFireState == RapidFireState.SPINUP_WAIT) {
                setSpindexShootPosition(rapidFireIndex);
                if (now >= rapidFireTimer) {
                    flicker.setPosition(flickerUp);
                    rapidFireTimer = now + 200;
                    rapidFireState = RapidFireState.FLICK_UP;
                }
            } else if (rapidFireState == RapidFireState.FLICK_UP) {
                if (now >= rapidFireTimer) {
                    flicker.setPosition(flickerDown);
                    slots[rapidFireIndex] = "unknown";

                    if (rapidFireIndex < 2 && anySlotLoaded()) {
                        rapidFireIndex++;
                        intakeBurstUntil = now + INTAKE_BURST_MS;
                        rapidFireTimer = now + 900;
                        rapidFireState = RapidFireState.SPINUP_WAIT;
                    } else {
                        rapidFireTimer = now + 200;
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
        }

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
        telemetry.addData("RapidFire", rapidFireState);
        telemetry.addData("Turret Power", turretPower);
        telemetry.addData("Turret Pos", turret.getCurrentPosition());
        telemetry.addData("Robot Pose", "X=%.1f Y=%.1f Heading=%.1f",
                robotPose.getX(), robotPose.getY(), Math.toDegrees(robotPose.getHeading()));
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

    private double feedforward(double targetVel) {
        return 0.4 * Math.signum(targetVel) + 0.01 * targetVel;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double getVelocityFromDistance(double distance) {
        for (int i = 0; i < POWER_TABLE.length - 1; i++) {
            if (distance >= POWER_TABLE[i][0] && distance <= POWER_TABLE[i + 1][0]) {
                double x0 = POWER_TABLE[i][0], y0 = POWER_TABLE[i][1];
                double x1 = POWER_TABLE[i + 1][0], y1 = POWER_TABLE[i + 1][1];
                return y0 + (distance - x0) / (x1 - x0) * (y1 - y0);
            }
        }
        if (distance < POWER_TABLE[0][0]) return POWER_TABLE[0][1];
        else return POWER_TABLE[POWER_TABLE.length - 1][1];
    }
}
