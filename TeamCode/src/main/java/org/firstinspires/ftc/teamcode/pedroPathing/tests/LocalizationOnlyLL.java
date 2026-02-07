package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

//@TeleOp(name = "Localization Only from LL")
public class LocalizationOnlyLL extends OpMode {

    private DcMotor turret;
    private Limelight3A limelight;
    private Follower follower;

    private static final double LIMELIGHT_X_OFFSET = -3.5; // inches left of shooter
    private static final double LL_CORRECTION_THRESHOLD = 5.0; // inches

    private static final double GOAL_X = 0;
    private static final double GOAL_Y = 144;

    private boolean llInitializedPose = false;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotor.Direction.REVERSE);

        // Keep turret at straight forward
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0)); // initial straight forward
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.getPoseTracker().getPose();

        LLResult result = limelight.getLatestResult();

        // -------------------- Use LL for localization --------------------
        if (result != null && result.isValid()) {
            double dx = GOAL_X - pose.getX();
            double dy = GOAL_Y - pose.getY();
            double distance = Math.hypot(dx, dy);

            // crude LL X/Y estimate from distance (can refine later)
            double llX = GOAL_X - distance;
            double llY = GOAL_Y;

            if (!llInitializedPose) {
                // first time LL sets the pose
                follower.getPoseTracker().setPose(new Pose(llX, llY, pose.getHeading()));
                llInitializedPose = true;
            } else {
                // ongoing corrections only if within threshold
                double delta = Math.hypot(llX - pose.getX(), llY - pose.getY());
                if (delta < LL_CORRECTION_THRESHOLD) {
                    follower.getPoseTracker().setPose(new Pose(
                            (pose.getX() + llX) / 2.0,
                            (pose.getY() + llY) / 2.0,
                            pose.getHeading()
                    ));
                }
            }
        }

        // -------------------- Keep turret still --------------------
        turret.setPower(0);

        // -------------------- Telemetry --------------------
        telemetry.addData("LL Initialized Pose", llInitializedPose);
        telemetry.addData("Turret Pos", turret.getCurrentPosition());
        telemetry.addData("Robot X", follower.getPoseTracker().getPose().getX());
        telemetry.addData("Robot Y", follower.getPoseTracker().getPose().getY());
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.setPower(0);
        limelight.stop();
    }
}
