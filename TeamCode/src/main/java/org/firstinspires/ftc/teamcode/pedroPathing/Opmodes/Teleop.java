package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@TeleOp(name = "TeleOp")
public class Teleop extends OpMode {
    private Follower follower;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        // Initialize your drive follower and localization
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();

    }

    @Override
    public void start() {
        // Start normal driver control mode
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update(); // Always update follower localization

        // === DRIVER CONTROL ===
        double driveY = gamepad1.left_stick_y;  // Forward/Backward
        double driveX = gamepad1.left_stick_x;  // Strafe
        double turn = -gamepad1.right_stick_x;   // Rotation

        if (!slowMode) {
            follower.setTeleOpDrive(
                    driveY,
                    driveX,
                    turn,
                    false
            );  // Robot-Centric
        } else {
            follower.setTeleOpDrive(
                    driveY * slowModeMultiplier,
                    driveX * slowModeMultiplier,
                    turn * slowModeMultiplier,
                    false
            );
        }

        // SLOW MODE TOGGLE
        if (gamepad1.right_bumper) {
            slowMode = true;
        } else {
            slowMode = false;
        }
    }
}
