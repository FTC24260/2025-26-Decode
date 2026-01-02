package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.ShooterSubsystem;

/**
 * ShooterCommand
 *
 * Auto-aims the turret based on robot's current pose from the Pedro Pathing Follower,
 * calculates required yaw, pitch, and flywheel speed, and fires automatically.
 */
public class ShooterCommand {

    private final ShooterSubsystem shooter;
    private final Follower follower;

    // Field constants (in inches)
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;
    private static final double GOAL_Z = 36.0;
    private static final double SHOOTER_Z = 12.0;
    private static final double VERTEX_Z = 60.0;

    // Computed targets
    private double targetPitch;   // radians
    private double targetYaw;     // radians
    private double shooterPower;  // 0–1

    private final ElapsedTime timer = new ElapsedTime();
    private final double shootDuration = 2.0; // seconds

    public ShooterCommand(ShooterSubsystem shooter, Follower follower) {
        this.shooter = shooter;
        this.follower = follower;
    }

    public void initialize() {
        // === 1 get current robot position from Pedro Pathing follower ===
        Pose pose = follower.getPose();
        double shooterX = pose.getX();
        double shooterY = pose.getY();

        // === 2 Calculate yaw (horizontal rotation) ===
        double deltaX = GOAL_X - shooterX;
        double deltaY = GOAL_Y - shooterY;
        targetYaw = Math.atan2(deltaY, deltaX); // radians

        // === 3 Horizontal distance to target ===
        double horizontalDistance = Math.hypot(deltaX, deltaY);

        // === 4 Calculate pitch angle using a parabolic arc ===
        double z0 = SHOOTER_Z;
        double zGoal = GOAL_Z;
        double xV = horizontalDistance / 2.0; // midpoint for vertex
        double a = (z0 - VERTEX_Z) / Math.pow(0 - xV, 2);
        double slope = 2 * a * (0 - xV);
        targetPitch = Math.atan(slope);

        // === 5 Command turret mechanisms ===
        shooter.setYawAngle(Math.toDegrees(targetYaw)); // Motor PID target
        shooter.setPitchPosition(targetPitchToServoPosition(targetPitch)); // Servo position

        // === 6 Compute flywheel speed based on distance and pitch ===
        double launchSpeed = computeLaunchSpeed(horizontalDistance, zGoal - z0, targetPitch);
        double wheelRadius = 2.362205; // inches
        shooterPower = launchSpeedToMotorPower(launchSpeed, wheelRadius);
        shooter.setShooterPower(shooterPower);

        timer.reset();
    }

    public void execute() {
        // Continuously adjust yaw via PID
        shooter.updateYaw();
    }

    public boolean isFinished() {
        // Finish once yaw is aligned and time has elapsed
        return timer.seconds() >= shootDuration && shooter.isYawAtTarget(2.0);
    }

    public void end() {
        shooter.stopShooter();
    }

    /* ---------- Helper Methods ---------- */
    private double computeLaunchSpeed(double distance, double heightDiff, double pitch) {
        // Basic ballistic model (no drag)
        double g = 386.09; // in/s^2
        double numerator = g * distance * distance;
        double denominator = 2 * Math.pow(Math.cos(pitch), 2)
                * (distance * Math.tan(pitch) - heightDiff);
        if (denominator <= 0) return 0;
        return Math.sqrt(numerator / denominator);
    }

    private double launchSpeedToMotorPower(double launchSpeed, double wheelRadius) {
        double wheelRPM = (launchSpeed / (2 * Math.PI * wheelRadius)) * 60.0;
        double maxRPM = 6000.0;
        double power = wheelRPM / maxRPM;
        return Math.min(Math.max(power, 0), 1.0);
    }

    private double targetPitchToServoPosition(double pitchRad) {
        double minAngle = 0.0;
        double maxAngle = Math.PI / 2;
        return Math.min(Math.max((pitchRad - minAngle) / (maxAngle - minAngle), 0), 1);
    }

}
