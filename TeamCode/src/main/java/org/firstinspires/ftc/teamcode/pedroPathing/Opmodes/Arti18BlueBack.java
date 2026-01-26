package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
public class Arti18BlueBack extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private DcMotor intake;
    private ColorSensor colorSensor;
    private Servo leftIndex, rightIndex;

    private final double[] intakePositions = {0.084, 0.174, 0.264};
    private final String[] slots = {"unknown", "unknown", "unknown"};
    private int currentIndex = 0;

    private long ignoreSensorUntil = 0;
    private static final long SENSOR_IGNORE_MS = 800;
    private long initialIgnoreUntil = 0;
    private static final long INITIAL_IGNORE_MS = 400;
    private boolean waitingForBallClear = false;
    private static final double SERVO_DEADZONE = 0.004;
    private double lastLeftIndexPos = -1;
    private double lastRightIndexPos = -1;

    private final Pose startPose = new Pose(22, 127, Math.toRadians(142));
    private final Pose pickup1SlowPose = new Pose(30, 84, Math.toRadians(180)); // original end
    private final Pose pickup1FastEnd = new Pose(44, 84, Math.toRadians(180));   // split point
    private final Pose shoot6Pose = new Pose(60, 84, Math.toRadians(200));
    private final Pose pickup2Pose = new Pose(25, 50, Math.toRadians(180));
    private final Pose shoot9Pose = new Pose(60, 84, Math.toRadians(150));
    private final Pose shoot12Pose = new Pose(60, 84, Math.toRadians(150));
    private final Pose shoot15Pose = new Pose(60, 84, Math.toRadians(180));
    private final Pose shoot18Pose = new Pose(60, 84, Math.toRadians(180));
    private final Pose pickupGate1Pose = new Pose(15, 57, Math.toRadians(150));
    private final Pose pickupGate2Pose = new Pose(15, 57, Math.toRadians(150));
    private final Pose pickupGate3Pose = new Pose(15, 57, Math.toRadians(150));

    private PathChain shootPreload3;
    private PathChain pickup1Fast;
    private PathChain pickup1Slow;
    private PathChain shoot6;
    private PathChain pickup2;
    private PathChain shoot9;
    private PathChain pickupGate1;
    private PathChain shoot12;
    private PathChain pickupGate2;
    private PathChain shoot15;
    private PathChain pickupGate3;
    private PathChain shoot18;

    public void preStart() {
        shootPreload3 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot6Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot6Pose.getHeading())
                .build();

        // Pickup1 split
        pickup1Fast = follower.pathBuilder()
                .addPath(new BezierLine(shoot6Pose, pickup1FastEnd))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pickup1Slow = follower.pathBuilder()
                .addPath(new BezierLine(pickup1FastEnd, pickup1SlowPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shoot6 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1SlowPose, shoot6Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot6Pose, pickup2Pose, pickup2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shoot9 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, new Pose(20.25, 56.5), shoot9Pose))
                .setConstantHeadingInterpolation(Math.toRadians(150))
                .build();

        pickupGate1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot9Pose, pickupGate1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(150))
                .build();

        shoot12 = follower.pathBuilder()
                .addPath(new BezierLine(pickupGate1Pose, shoot12Pose))
                .setConstantHeadingInterpolation(Math.toRadians(150))
                .build();

        pickupGate2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot12Pose, pickupGate2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(150))
                .build();

        shoot15 = follower.pathBuilder()
                .addPath(new BezierLine(pickupGate2Pose, shoot15Pose))
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))
                .build();

        pickupGate3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot15Pose, pickupGate3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(150))
                .build();

        shoot18 = follower.pathBuilder()
                .addPath(new BezierLine(pickupGate3Pose, shoot18Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(shootPreload3, true);
                    setPathState(1);
                }
                break;

            case 1: // pickup1 fast
                if (!follower.isBusy()) {
                    intake.setPower(-1); // run intake
                    initialIgnoreUntil = System.currentTimeMillis() + INITIAL_IGNORE_MS;
                    follower.followPath(pickup1Fast, true);
                    setPathState(2);
                }
                break;

            case 2: // pickup1 slow
                if (!follower.isBusy()) {
                    follower.followPath(pickup1Slow, true);
                    setPathState(3);
                }
                break;

            case 3: // shoot6
                if (!follower.isBusy()) {
                    intake.setPower(0); // stop intake
                    follower.followPath(shoot6, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) { follower.followPath(pickup2, true); setPathState(5); }
                break;
            case 5:
                if (!follower.isBusy()) { follower.followPath(shoot9, true); setPathState(6); }
                break;
            case 6:
                if (!follower.isBusy()) { follower.followPath(pickupGate1, true); setPathState(7); }
                break;
            case 7:
                if (!follower.isBusy()) { follower.followPath(shoot12, true); setPathState(8); }
                break;
            case 8:
                if (!follower.isBusy()) { follower.followPath(pickupGate2, true); setPathState(9); }
                break;
            case 9:
                if (!follower.isBusy()) { follower.followPath(shoot15, true); setPathState(10); }
                break;
            case 10:
                if (!follower.isBusy()) { follower.followPath(pickupGate3, true); setPathState(11); }
                break;
            case 11:
                if (!follower.isBusy()) { follower.followPath(shoot18, true); setPathState(12); }
                break;
            case 12:
                if (!follower.isBusy()) { setPathState(-1); }
                break;
        }
    }

    private void updateSpindexLogic() {
        if (pathState != 1 && pathState != 2) return; // only during pickup1 segments

        long now = System.currentTimeMillis();

        String detectedColor = detectColor();
        if (detectedColor.equals("unknown")) waitingForBallClear = false;

        if (!waitingForBallClear
                && !detectedColor.equals("unknown")
                && now >= ignoreSensorUntil
                && now >= initialIgnoreUntil
                && currentIndex < 3) {
            slots[currentIndex] = detectedColor;
            currentIndex++;
            setSpindexIntakePosition(currentIndex);
            ignoreSensorUntil = now + SENSOR_IGNORE_MS;
            waitingForBallClear = true;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        intake = hardwareMap.get(DcMotor.class, "intake");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");

        setSpindexIntakePosition(0);

        preStart();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        updateSpindexLogic();
        publishTelemetry();
    }

    public void publishTelemetry() {
        telemetry.addData("Path State", pathState);
        telemetry.addData("Slots", slots[0] + ", " + slots[1] + ", " + slots[2]);
        Pose p = follower.getPose();
        telemetry.addData("X", p.getX());
        telemetry.addData("Y", p.getY());
        telemetry.addData("Heading", Math.toDegrees(p.getHeading()));
        telemetry.update();
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
        double left = Math.max(0, Math.min(1, pos));
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

    @Override
    public void stop() {
        intake.setPower(0);
    }
}
