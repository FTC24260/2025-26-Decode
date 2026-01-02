package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;
import com.bylazar.telemetry.TelemetryManager;

public class DriveSubsystem {

    private final Follower follower;
    private final Telemetry telemetry;
    private final TelemetryManager telemetryM;
    private boolean automatedDrive = false;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetryM = com.bylazar.telemetry.PanelsTelemetry.INSTANCE.getTelemetry();

        // Create the follower the same way Teleop does
        this.follower = Constants.createFollower(hardwareMap);

        // Initialize pose to (0,0,0) unless Teleop sets startingPose before init
        this.follower.setStartingPose(new Pose());
        this.follower.update();
    }

    /**
     * Call once per main loop (opmode.loop) to update internal localizer
     */
    public void update() {
        follower.update();
        telemetryM.update();
        // Mirror debug telemetry similar to Teleop
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.update();
    }

    /* ---------- Pose / Odometry access ---------- */

    public Pose getPose() { return follower.getPose(); }

    public double getX() { return follower.getPose().getX(); }

    public double getY() { return follower.getPose().getY(); }

    public double getHeading() { return follower.getPose().getHeading(); }

    public void setPoseEstimate(Pose p) {
        follower.setStartingPose(p);
        follower.update();
    }

    /* ---------- Teleop drive passthrough (wraps follower.setTeleOpDrive) ---------- */

    /**
     * Teleop drive wrapper.
     *
     * @param forward forward/backward input (-1..1)
     * @param strafe  left/right input (-1..1)
     * @param turn    rotation input (-1..1)
     * @param robotCentric true -> robot-centric, false -> field-centric
     */
    public void teleopDrive(double forward, double strafe, double turn, boolean robotCentric) {
        follower.setTeleOpDrive(forward, strafe, turn, robotCentric);
    }

    /**
     * Start teleop drive (brake/float mode behavior handled by follower/Constants).
     * This switches the follower back to driver control if it was following a path.
     */
    public void startTeleopDrive() {
        follower.startTeleopDrive();
        automatedDrive = false;
    }

    /* ---------- Automated path following helpers ---------- */

    /**
     * Begin following a path chain (non-blocking).
     * @param pathChain the PathChain to follow (you can build it lazily with a Supplier in caller)
     */
    public void followPath(PathChain pathChain) {
        follower.followPath(pathChain);
        automatedDrive = true;
    }

    /**
     * Returns true while the follower is still executing a path.
     */
    public boolean isBusy() {
        return follower.isBusy();
    }

    /**
     * Stop following and return to teleop control immediately.
     */
    public void cancelPathFollowing() {
        follower.startTeleopDrive();
        automatedDrive = false;
    }

    /* ---------- Convenience debug ---------- */

    public boolean isAutomatedDrive() { return automatedDrive; }
}
