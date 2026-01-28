package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@Autonomous(name = "Arti9 Blue Front Auto + Spindex + Turret", group = "Autonomous")
public class Arti9BlueFront extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private DcMotor intake;
    private ColorSensor colorSensor;
    private Servo leftIndex, rightIndex;

    private DcMotor turret;
    private Limelight3A limelight;

    private final double[] intakePositions = {0.084, 0.174, 0.264};
    private final String[] slots = {"unknown", "unknown", "unknown"};
    private int currentIndex = 0;

    private long ignoreSensorUntil = 0;
    private static final long SENSOR_IGNORE_MS = 500;

    private long initialIgnoreUntil = 0;
    private static final long INITIAL_IGNORE_MS = 400;

    private boolean waitingForBallClear = false;

    private static final double SERVO_DEADZONE = 0.004;
    private double lastLeftIndexPos = -1;
    private double lastRightIndexPos = -1;

    // ==== Poses ====
    private final Pose startPose = new Pose(13, 127, Math.toRadians(145));
    private final Pose shootPreloadPose = new Pose(60, 84, Math.toRadians(180));

    private final Pose pickup11Pose = new Pose(38, 84, Math.toRadians(180));
    private final Pose pickup12Pose = new Pose(33, 84, Math.toRadians(180));
    private final Pose pickup13Pose = new Pose(28, 84, Math.toRadians(180));
    private final Pose shoot6Pose = new Pose(60, 84, Math.toRadians(180));

    private final Pose pickup21Control = new Pose(60, 64);
    private final Pose pickup21Pose = new Pose(38, 60, Math.toRadians(180));
    private final Pose pickup22Pose = new Pose(33, 60, Math.toRadians(180));
    private final Pose pickup23Pose = new Pose(28, 60, Math.toRadians(180));
    private final Pose shoot9Pose = new Pose(60, 84, Math.toRadians(180));

    private final Pose pickup31Control = new Pose(60, 34);
    private final Pose pickup31Pose = new Pose(38, 36, Math.toRadians(180));
    private final Pose pickup32Pose = new Pose(33, 36, Math.toRadians(180));
    private final Pose pickup33Pose = new Pose(28, 36, Math.toRadians(180));
    private final Pose shoot12Pose = new Pose(60, 84, Math.toRadians(180));

    // ==== Paths ====
    private PathChain shootPreload, pickup11, pickup12, pickup13, shoot6,
            pickup21, pickup22, pickup23, shoot9,
            pickup31, pickup32, pickup33, shoot12;

    // ==== Turret constants ====
    private final int TURRET_MAX = 510;
    private final int TURRET_MIN = -350;
    private final double MAX_TURRET_POWER = 0.6;
    private final double TURRET_KP = 0.021;
    private int turretZero = 0;

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        intake = hardwareMap.get(DcMotor.class, "intake");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretZero = turret.getCurrentPosition();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        setSpindexIntakePosition(0);
        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
        initialIgnoreUntil = System.currentTimeMillis() + INITIAL_IGNORE_MS;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        updateSpindexAuto();
        updateTurretAlways();

        Pose p = follower.getPose();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Slots", slots[0] + ", " + slots[1] + ", " + slots[2]);
        telemetry.addData("X", p.getX());
        telemetry.addData("Y", p.getY());
        telemetry.addData("Heading", Math.toDegrees(p.getHeading()));
        telemetry.addData("Turret Pos", turret.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0);
        turret.setPower(0);
        limelight.stop();
    }

    // ===== Turret always aim =====
    private void updateTurretAlways() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double dx = 0 - follower.getPose().getX();
            double dy = 144 - follower.getPose().getY();
            double targetAngle = Math.atan2(dy, dx) - follower.getPose().getHeading();

            double ticksPerRadian = (TURRET_MAX - TURRET_MIN) / (2 * Math.PI);
            int targetTicks = turretZero + (int) (targetAngle * ticksPerRadian);

            int delta = targetTicks - turret.getCurrentPosition();
            int maxRange = TURRET_MAX - TURRET_MIN;
            while (delta > maxRange / 2) delta -= maxRange;
            while (delta < -maxRange / 2) delta += maxRange;

            double power = TURRET_KP * delta;
            power = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, power));
            if ((turret.getCurrentPosition() >= TURRET_MAX && power > 0) ||
                    (turret.getCurrentPosition() <= TURRET_MIN && power < 0)) power = 0;

            turret.setPower(power);
        }
    }

    // ===== Autonomous paths =====
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: if (!follower.isBusy()) { follower.followPath(shootPreload, true); setPathState(1); } break;
            case 1: if (!follower.isBusy()) { intake.setPower(-1); initialIgnoreUntil = System.currentTimeMillis() + INITIAL_IGNORE_MS; follower.followPath(pickup11, true); setPathState(2); } break;
            case 2: if (!follower.isBusy()) { follower.followPath(pickup12, true); setPathState(3); } break;
            case 3: if (!follower.isBusy()) { follower.followPath(pickup13, true); setPathState(4); } break;
            case 4: if (!follower.isBusy()) { intake.setPower(0); follower.followPath(shoot6, true); setPathState(5); } break;
            case 5: if (!follower.isBusy()) { intake.setPower(-1); initialIgnoreUntil = System.currentTimeMillis() + INITIAL_IGNORE_MS; follower.followPath(pickup21, true); setPathState(6); } break;
            case 6: if (!follower.isBusy()) { follower.followPath(pickup22, true); setPathState(7); } break;
            case 7: if (!follower.isBusy()) { follower.followPath(pickup23, true); setPathState(8); } break;
            case 8: if (!follower.isBusy()) { intake.setPower(0); follower.followPath(shoot9, true); setPathState(9); } break;
            case 9: if (!follower.isBusy()) { intake.setPower(-1); initialIgnoreUntil = System.currentTimeMillis() + INITIAL_IGNORE_MS; follower.followPath(pickup31, true); setPathState(10); } break;
            case 10: if (!follower.isBusy()) { follower.followPath(pickup32, true); setPathState(11); } break;
            case 11: if (!follower.isBusy()) { follower.followPath(pickup33, true); setPathState(12); } break;
            case 12: if (!follower.isBusy()) { intake.setPower(0); follower.followPath(shoot12, true); setPathState(13); } break;
        }
    }

    // ===== Spindex =====
    private void updateSpindexAuto() {
        long now = System.currentTimeMillis();
        String detectedColor = detectColor();
        if (detectedColor.equals("unknown")) waitingForBallClear = false;

        if (!waitingForBallClear && !detectedColor.equals("unknown") && now >= ignoreSensorUntil
                && now >= initialIgnoreUntil && currentIndex < 3) {
            slots[currentIndex] = detectedColor;
            currentIndex++;
            setSpindexIntakePosition(currentIndex);
            ignoreSensorUntil = now + SENSOR_IGNORE_MS;
            waitingForBallClear = true;
        }
    }

    private String detectColor() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        if (g > 1.2 * r && g > 1.2 * b && g > 20) return "green";
        int maxRB = Math.max(r, b), minRB = Math.min(r, b);
        if (maxRB > 40 && minRB >= 0.5 * maxRB && g < 0.7 * maxRB) return "purple";
        return "unknown";
    }

    private void setSpindexIntakePosition(int index) {
        if (index >= intakePositions.length) index = intakePositions.length - 1;
        applyServoDeadzone(intakePositions[index]);
    }

    private void applyServoDeadzone(double pos) {
        double left = Math.max(0, Math.min(1, pos));
        double right = left;
        if (Math.abs(left - lastLeftIndexPos) > SERVO_DEADZONE) { leftIndex.setPosition(left); lastLeftIndexPos = left; }
        if (Math.abs(right - lastRightIndexPos) > SERVO_DEADZONE) { rightIndex.setPosition(right); lastRightIndexPos = right; }
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    private void buildPaths() {
        shootPreload = follower.pathBuilder().addPath(new BezierLine(startPose, shootPreloadPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPreloadPose.getHeading()).build();

        pickup11 = follower.pathBuilder().addPath(new BezierLine(shootPreloadPose, pickup11Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();

        pickup12 = follower.pathBuilder().addPath(new BezierLine(pickup11Pose, pickup12Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();

        pickup13 = follower.pathBuilder().addPath(new BezierLine(pickup12Pose, pickup13Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();

        shoot6 = follower.pathBuilder().addPath(new BezierLine(pickup13Pose, shoot6Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();

        pickup21 = follower.pathBuilder().addPath(new BezierCurve(shoot6Pose, pickup21Control, pickup21Pose))
                .setLinearHeadingInterpolation(shoot6Pose.getHeading(), pickup21Pose.getHeading()).build();

        pickup22 = follower.pathBuilder().addPath(new BezierLine(pickup21Pose, pickup22Pose))
                .setConstantHeadingInterpolation(pickup21Pose.getHeading()).build();

        pickup23 = follower.pathBuilder().addPath(new BezierLine(pickup22Pose, pickup23Pose))
                .setConstantHeadingInterpolation(pickup21Pose.getHeading()).build();

        shoot9 = follower.pathBuilder().addPath(new BezierLine(pickup23Pose, shoot9Pose))
                .setConstantHeadingInterpolation(pickup21Pose.getHeading()).build();

        pickup31 = follower.pathBuilder().addPath(new BezierCurve(shoot9Pose, pickup31Control, pickup31Pose))
                .setConstantHeadingInterpolation(pickup21Pose.getHeading()).build();

        pickup32 = follower.pathBuilder().addPath(new BezierLine(pickup31Pose, pickup32Pose))
                .setConstantHeadingInterpolation(pickup21Pose.getHeading()).build();

        pickup33 = follower.pathBuilder().addPath(new BezierLine(pickup32Pose, pickup33Pose))
                .setConstantHeadingInterpolation(pickup21Pose.getHeading()).build();

        shoot12 = follower.pathBuilder().addPath(new BezierLine(pickup33Pose, shoot12Pose))
                .setConstantHeadingInterpolation(pickup21Pose.getHeading()).build();
    }
}
