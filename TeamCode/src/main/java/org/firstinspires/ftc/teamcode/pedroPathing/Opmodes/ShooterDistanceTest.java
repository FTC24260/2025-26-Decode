package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "SingleBallTest")
public class ShooterDistanceTest extends OpMode {

    private DcMotor intake;
    private DcMotorEx shooterL, shooterR;
    private DcMotor turret;
    private Servo leftIndex, rightIndex;
    private Limelight3A limelight;

    private final double[] shootPositions = {0.31, 0.4, 0.49};
    private final double intakePosition = 0.084;

    private boolean lastA = false;
    private boolean ballLoaded = false;

    private int currentShootIndex = 0;

    private final double MAX_SHOOT_POWER = 0.7;
    private final double MIN_SHOOT_POWER = 0.4;

    private final double TURRET_MAX = 510;
    private final double TURRET_MIN = -350;
    private double Kp_GOAL = 0.021;
    private double goalX = 0;
    private double goalY = 144;
    private int turretZero = 0;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");

        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        shooterR.setDirection(REVERSE);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);

        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        turret.setDirection(REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretZero = turret.getCurrentPosition();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        setIndexIntake();
    }

    @Override
    public void loop() {
        // --- Intake control ---
        if (gamepad1.left_trigger > 0.1) {
            intake.setPower(-1);
            // Move spindex to shoot position
            setIndexShoot(currentShootIndex);
            ballLoaded = true;
        } else {
            intake.setPower(0);
        }

        // --- Shoot on A ---
        if (ballLoaded && gamepad1.a && !lastA) {
            // Get distance from limelight
            double distance = getLimelightDistance();
            double power = mapDistanceToPower(distance);
            shooterL.setPower(power);
            shooterR.setPower(power);

            // Wait a short time to simulate firing
            sleep(300);

            // Stop shooter and return spindex
            shooterL.setPower(0);
            shooterR.setPower(0);
            setIndexIntake();
            ballLoaded = false;
            currentShootIndex = (currentShootIndex + 1) % shootPositions.length;
        }
        lastA = gamepad1.a;

        // --- Turret aiming ---
        double dx = goalX;
        double dy = goalY;
        double targetAngle = Math.atan2(dy, dx); // simplified for test
        double ticksPerRadian = (TURRET_MAX - TURRET_MIN) / (2 * Math.PI);
        int targetTicks = turretZero + (int)(targetAngle * ticksPerRadian);
        int delta = targetTicks - turret.getCurrentPosition();
        double turretPower = Kp_GOAL * delta;
        turretPower = clamp(turretPower, -0.6, 0.6);
        turret.setPower(turretPower);

        telemetry.addData("Ball Loaded", ballLoaded);
        telemetry.addData("Shooter Power", shooterL.getPower());
        telemetry.addData("Turret Pos", turret.getCurrentPosition());
        telemetry.addData("Distance", getLimelightDistance());
        telemetry.update();
    }

    private void setIndexShoot(int index) {
        if (index >= shootPositions.length) index = shootPositions.length - 1;
        leftIndex.setPosition(shootPositions[index]);
        rightIndex.setPosition(shootPositions[index]);
    }

    private void setIndexIntake() {
        leftIndex.setPosition(intakePosition);
        rightIndex.setPosition(intakePosition);
    }

    private double getLimelightDistance() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return 50; // default distance if no target
        }

        // Vertical angle to target (degrees)
        double ty = result.getTy();

        // Example heights in inches
        double cameraHeight = 8; // height of camera from floor
        double goalHeight = 24;  // height of target from floor

        double angleRad = Math.toRadians(ty);
        // Simple trigonometry: distance = height difference / tan(angle)
        return (goalHeight - cameraHeight) / Math.tan(angleRad);
    }



    private double mapDistanceToPower(double distance) {
        // Linear mapping between MIN_SHOOT_POWER and MAX_SHOOT_POWER
        double power = MIN_SHOOT_POWER + (MAX_SHOOT_POWER - MIN_SHOOT_POWER) * (distance / 100);
        return clamp(power, MIN_SHOOT_POWER, MAX_SHOOT_POWER);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private void sleep(long ms) {
        long end = System.currentTimeMillis() + ms;
        while (System.currentTimeMillis() < end) {}
    }

    @Override
    public void stop() {
        intake.setPower(0);
        shooterL.setPower(0);
        shooterR.setPower(0);
        turret.setPower(0);
        limelight.stop();
    }
}
