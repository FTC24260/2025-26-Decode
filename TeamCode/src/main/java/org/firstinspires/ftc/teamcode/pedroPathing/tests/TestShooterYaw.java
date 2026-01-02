package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.ShooterSubsystem;

@TeleOp(name = "Shooter Yaw Test")
public class TestShooterYaw extends OpMode {

    private Follower follower;
    private ShooterSubsystem shooter;

    // Field goal coordinates (in inches)
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();

        // Initialize ShooterSubsystem (yaw motor, pitch servo, shooter motor)
        shooter = new ShooterSubsystem(hardwareMap);

        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update localization
        follower.update();
        Pose pose = follower.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();

        // Calculate yaw to goal
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;
        double targetYawRad = Math.atan2(deltaY, deltaX);
        double targetYawDeg = Math.toDegrees(targetYawRad);

        // Command shooter yaw motor to this angle
        shooter.setYawAngle(targetYawDeg);  // PID-controlled

        // Run yaw PID update
        shooter.updateYaw();

        // Telemetry
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Target Yaw (deg)", targetYawDeg);
        telemetry.update();
    }
}
