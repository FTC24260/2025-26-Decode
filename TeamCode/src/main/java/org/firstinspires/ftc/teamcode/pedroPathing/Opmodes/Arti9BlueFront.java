package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@Autonomous(name = "Arti9 Blue Front Auto - Corrected Rapid Fire", group = "Autonomous")
public class Arti9BlueFront extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private DcMotorEx intake;
    private DcMotorEx shooterR, shooterL;
    private ColorSensor colorSensor;
    private Servo leftIndex, rightIndex, flicker;
    private DcMotor turret;

    private final double[] intakePositions = {0.084, 0.174, 0.264};
    private final double[] shootPositions = {0.31, 0.4, 0.49};
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

    private static final double VELOCITY_TOLERANCE = 20;
    private static final double flickerUp = 0.4;
    private static final double flickerDown = 0.7;

    private final int TURRET_MAX = 510;
    private final int TURRET_MIN = -350;
    private final double MAX_POWER_GOAL = 0.6;
    private final double Kp_GOAL = 0.01;
    private int turretZero = 0;
    private final double goalX = 0;
    private final double goalY = 144;

    private long rapidFireTimer = 0;
    private enum RapidFireState {IDLE, FLICK1_UP, FLICK1_DOWN, SPINDEX2_WAIT, FLICK2_UP, FLICK2_DOWN, SPINDEX3_WAIT, FLICK3_UP, FLICK3_DOWN, DONE}
    private RapidFireState rapidFireState = RapidFireState.IDLE;

    // ==== Poses ====
    private final Pose startPose = new Pose(13, 127, Math.toRadians(145));
    private final Pose shootPreloadPose = new Pose(60, 84, Math.toRadians(180));
    private final Pose pickup11Pose = new Pose(38, 84, Math.toRadians(180));
    private final Pose pickup12Pose = new Pose(31, 84, Math.toRadians(180));
    private final Pose pickup13Pose = new Pose(26, 84, Math.toRadians(180));
    private final Pose shoot6Pose = new Pose(60, 84, Math.toRadians(180));

    private final Pose pickup21Control = new Pose(60, 64);
    private final Pose pickup21Pose = new Pose(38, 60, Math.toRadians(180));
    private final Pose pickup22Pose = new Pose(32, 60, Math.toRadians(180));
    private final Pose pickup23Pose = new Pose(20, 60, Math.toRadians(180));
    private final Pose shoot9Pose = new Pose(60, 84, Math.toRadians(180));

    private final Pose pickup31Control = new Pose(60, 34);
    private final Pose pickup31Pose = new Pose(38, 36, Math.toRadians(180));
    private final Pose pickup32Pose = new Pose(32, 36, Math.toRadians(180));
    private final Pose pickup33Pose = new Pose(20, 36, Math.toRadians(180));
    private final Pose shoot12Pose = new Pose(60, 84, Math.toRadians(180));

    // ==== Paths ====
    private PathChain shootPreload, pickup11, pickup12, pickup13, shoot6,
            pickup21, pickup22, pickup23, shoot9,
            pickup31, pickup32, pickup33, shoot12;

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");
        flicker = hardwareMap.get(Servo.class, "flicker");

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretZero = turret.getCurrentPosition();

        applyServoDeadzone(intakePositions[0]);
        flicker.setPosition(flickerDown);

        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        pathState = 0;
        rapidFireState = RapidFireState.IDLE;
        initialIgnoreUntil = System.currentTimeMillis() + INITIAL_IGNORE_MS;
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        // ================= Spindex Intake =================
        String detectedColor = detectColor();
        if (detectedColor.equals("unknown")) waitingForBallClear = false;

        if (!waitingForBallClear && !detectedColor.equals("unknown") && now >= ignoreSensorUntil && currentIndex < 3) {
            slots[currentIndex] = detectedColor;
            currentIndex++;
            setSpindexIntakePosition(currentIndex, false); // intake positions
            ignoreSensorUntil = now + SENSOR_IGNORE_MS;
            waitingForBallClear = true;
        }

        // ================= Path + Intake =================
        switch (pathState) {
            case 0:
                if (!follower.isBusy() && rapidFireState == RapidFireState.IDLE) {
                    follower.followPath(shootPreload, true);
                    pathState = 1;
                    updateShootingSequence(now);
                }
                break;
            case 1:
                intake.setPower(0);
                if (rapidFireState == RapidFireState.DONE && !follower.isBusy()) { follower.followPath(pickup11,true); pathState=2; } break;
            case 2: intake.setPower(-1); if (!follower.isBusy()) { follower.followPath(pickup12,true); pathState=3; } break;
            case 3: intake.setPower(-1); if (!follower.isBusy()) { follower.followPath(pickup13,true); pathState=4; } break;
            case 4: intake.setPower(-1); if (!follower.isBusy()) { follower.followPath(shoot6,true); pathState=5; updateShootingSequence(now); } break;
            case 5: intake.setPower(0); if (rapidFireState == RapidFireState.DONE && !follower.isBusy()) { follower.followPath(pickup21,true); pathState=6; } break;
            case 6: intake.setPower(-1); if (!follower.isBusy()) { follower.followPath(pickup22,true); pathState=7; } break;
            case 7: intake.setPower(-1); if (!follower.isBusy()) { follower.followPath(pickup23,true); pathState=8; } break;
            case 8: intake.setPower(-1); if (!follower.isBusy()) { follower.followPath(shoot9,true); pathState=9; updateShootingSequence(now); } break;
            case 9: intake.setPower(0); if (rapidFireState == RapidFireState.DONE && !follower.isBusy()) { follower.followPath(pickup31,true); pathState=10; } break;
            case 10: intake.setPower(-1); if (!follower.isBusy()) { follower.followPath(pickup32,true); pathState=11; } break;
            case 11: intake.setPower(-1); if (!follower.isBusy()) { follower.followPath(pickup33,true); pathState=12; } break;
            case 12: intake.setPower(-1); if (!follower.isBusy()) { follower.followPath(shoot12,true); pathState=13; updateShootingSequence(now); } break;
            case 13: intake.setPower(0); if (rapidFireState == RapidFireState.DONE && !follower.isBusy()) { pathState=14; } break;
        }


        follower.update();

        // ================= Turret =================
        Pose robotPose = follower.getPose();
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

        telemetry.addData("PathState", pathState);
        telemetry.addData("Slots", slots[0]+", "+slots[1]+", "+slots[2]);
        telemetry.addData("TurretPos", turret.getCurrentPosition());
        telemetry.addData("RapidFire", rapidFireState);
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0);
        shooterR.setPower(0);
        shooterL.setPower(0);
        turret.setPower(0);
    }

    // ================= Rapid Fire After Path 0 =================
    private enum ShootState {
        IDLE,
        RAMPING,
        WAIT_VELOCITY,
        FLICK1_UP, FLICK1_DOWN, SPINDEX1_WAIT,
        FLICK2_UP, FLICK2_DOWN, SPINDEX2_WAIT,
        FLICK3_UP, FLICK3_DOWN,
        DONE
    }

    private ShootState shootState = ShootState.IDLE;
    private long shootTimer = 0;

    private void startShootingSequence() {
        shooterR.setVelocity(getTargetShooterVelocity()); // you can adjust with distance if you want
        shooterL.setVelocity(getTargetShooterVelocity());
        shootState = ShootState.WAIT_VELOCITY;
    }

    // Call this inside loop()
    private void updateShootingSequence(long now) {
        switch (shootState) {
            case IDLE: break;

            case WAIT_VELOCITY:
                shooterR.setVelocity(getTargetShooterVelocity());
                shooterL.setVelocity(getTargetShooterVelocity());
                if (Math.abs(shooterR.getVelocity() - getTargetShooterVelocity()) < VELOCITY_TOLERANCE) {
                    flicker.setPosition(flickerUp);
                    shootTimer = now + 200;
                    shootState = ShootState.FLICK1_UP;
                }
                break;

            case FLICK1_UP:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerDown);
                    shootTimer = now + 200;
                    shootState = ShootState.FLICK1_DOWN;
                }
                break;

            case FLICK1_DOWN:
                if (now >= shootTimer) {
                    setSpindexPosition(1); // move spindex
                    shootTimer = now + 500;
                    shootState = ShootState.SPINDEX1_WAIT;
                }
                break;

            case SPINDEX1_WAIT:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerUp);
                    shootTimer = now + 200;
                    shootState = ShootState.FLICK2_UP;
                }
                break;

            case FLICK2_UP:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerDown);
                    shootTimer = now + 200;
                    shootState = ShootState.FLICK2_DOWN;
                }
                break;

            case FLICK2_DOWN:
                if (now >= shootTimer) {
                    setSpindexPosition(2); // move spindex
                    shootTimer = now + 500;
                    shootState = ShootState.SPINDEX2_WAIT;
                }
                break;

            case SPINDEX2_WAIT:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerUp);
                    shootTimer = now + 200;
                    shootState = ShootState.FLICK3_UP;
                }
                break;

            case FLICK3_UP:
                if (now >= shootTimer) {
                    flicker.setPosition(flickerDown);
                    shootTimer = now + 200;
                    shootState = ShootState.FLICK3_DOWN;
                }
                break;

            case FLICK3_DOWN:
                if (now >= shootTimer) {
                    shooterR.setPower(0);
                    shooterL.setPower(0);
                    shootState = ShootState.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    private void setSpindexPosition(int index) {
        if (index >= shootPositions.length) index = shootPositions.length-1;
        leftIndex.setPosition(shootPositions[index]);
        rightIndex.setPosition(shootPositions[index]);
    }

    private double getTargetShooterVelocity() {
        // you can replace this with distance-based calculation if needed
        return 1900;
    }


    // ================= Spindex =================
    private String detectColor() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        if (g > 1.2*r && g > 1.2*b && g>20) return "green";
        int maxRB=Math.max(r,b), minRB=Math.min(r,b);
        if(maxRB>40 && minRB>=0.5*maxRB && g<0.7*maxRB) return "purple";
        return "unknown";
    }

    private void setSpindexIntakePosition(int index, boolean shooting) {
        double[] positions = shooting ? shootPositions : intakePositions;
        if (index >= positions.length) index = positions.length-1;
        applyServoDeadzone(positions[index]);
    }

    private void applyServoDeadzone(double pos) {
        double left = Math.max(0, Math.min(1,pos));
        double right = left;
        if(Math.abs(left - lastLeftIndexPos)>SERVO_DEADZONE){ leftIndex.setPosition(left); lastLeftIndexPos=left; }
        if(Math.abs(right - lastRightIndexPos)>SERVO_DEADZONE){ rightIndex.setPosition(right); lastRightIndexPos=right; }
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
