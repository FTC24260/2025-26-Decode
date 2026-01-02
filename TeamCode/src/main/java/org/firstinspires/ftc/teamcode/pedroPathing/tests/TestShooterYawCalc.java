package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@TeleOp(name = "Shooter Yaw Calculation Test")
public class TestShooterYawCalc extends OpMode {

    // Pedro Pathing follower for localization
    private Follower follower;

    // Field goal coordinates (inches)
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing Follower...");
        telemetry.update();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();

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

        // Telemetry output
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Target Yaw (rad)", targetYawRad);
        telemetry.addData("Target Yaw (deg)", targetYawDeg);
        telemetry.update();
    }
}
