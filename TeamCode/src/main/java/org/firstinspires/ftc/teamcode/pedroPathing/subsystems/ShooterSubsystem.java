package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.RobotConstants;

public class ShooterSubsystem {

    private final DcMotor shooterMotor;      // flywheel
    private final DcMotor yawMotor;          // rotates turret
    private final Servo pitchServoLeft;      // left hood servo
    private final Servo pitchServoRight;     // right hood servo (mirrored)

    // PID constants for yaw
    private final double kP = RobotConstants.Shooter.KP;
    private final double kI = RobotConstants.Shooter.KI;
    private final double kD = RobotConstants.Shooter.KD;

    private double targetYawAngle = 0;
    private double lastError = 0;
    private double integral = 0;

    // Hood pitch limits (normalized servo range)
    private final double pitchMin = 0.0;   // straight up
    private final double pitchMax = 1.0;   // max down

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, RobotConstants.Hardware.SHOOTER_MOTOR);
        yawMotor = hardwareMap.get(DcMotor.class, RobotConstants.Hardware.TURRET_MOTOR);

        pitchServoLeft  = hardwareMap.get(Servo.class, RobotConstants.Hardware.HOOD_LEFT);
        pitchServoRight = hardwareMap.get(Servo.class, RobotConstants.Hardware.HOOD_RIGHT);

        // Flywheel motor setup
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Yaw motor setup
        yawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start both pitch servos at middle
        setPitchPosition(0.5);
    }

    /* ---------- Shooter wheel control ---------- */

    public void setShooterPower(double power) {
        shooterMotor.setPower(power);
    }

    public void stopShooter() {
        shooterMotor.setPower(0);
    }

    public double getShooterPower() {
        return shooterMotor.getPower();
    }

    /* ---------- Pitch control (mirrored left & right) ---------- */

    public void setPitchPosition(double position) {
        // Clamp input
        position = Math.min(Math.max(position, pitchMin), pitchMax);

        // Left servo direct
        pitchServoLeft.setPosition(position);

        // Right servo mirrored
        pitchServoRight.setPosition(Math.abs(1.0 - position));
    }

    public double getPitchPosition() {
        return pitchServoLeft.getPosition();
    }

    /* ---------- Yaw control (PID) ---------- */

    public void setYawAngle(double angle) {
        targetYawAngle = angle;
    }

    public void updateYaw() {
        double currentYawTicks = yawMotor.getCurrentPosition();

        // TODO: replace 1:1 with your real ticks→degrees conversion
        double currentAngle = currentYawTicks;

        double error = targetYawAngle - currentAngle;
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double output = (kP * error) + (kI * integral) + (kD * derivative);
        output = Math.max(Math.min(output, 1.0), -1.0);  // clamp

        yawMotor.setPower(output);
    }

    public boolean isYawAtTarget(double toleranceDegrees) {
        double currentAngle = yawMotor.getCurrentPosition();
        return Math.abs(currentAngle - targetYawAngle) <= toleranceDegrees;
    }
}
