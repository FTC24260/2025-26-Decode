package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

//@TeleOp(name = "Shooter Pitch Calculation Test")
public class TestShooterPitchCalc extends OpMode {

    private Follower follower;

    // Field & shooter constants (in inches)
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;
    private static final double GOAL_Z = 36.0;
    private static final double SHOOTER_Z = 12.0;
    private static final double VERTEX_Z = 60.0;

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

        // Calculate horizontal distance to goal
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;
        double horizontalDistance = Math.hypot(deltaX, deltaY);

        // Parabola math for pitch
        double x1 = 0; // relative horizontal shooter origin
        double xV = horizontalDistance / 2.0; // vertex x-coordinate
        double a = (SHOOTER_Z - VERTEX_Z) / Math.pow(x1 - xV, 2);
        double slope = 2 * a * (x1 - xV);
        double pitchRad = Math.atan(slope);
        double pitchDeg = Math.toDegrees(pitchRad);

        // Telemetry output
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Horizontal Distance", horizontalDistance);
        telemetry.addData("Pitch (rad)", pitchRad);
        telemetry.addData("Pitch (deg)", pitchDeg);
        telemetry.update();
    }
}
