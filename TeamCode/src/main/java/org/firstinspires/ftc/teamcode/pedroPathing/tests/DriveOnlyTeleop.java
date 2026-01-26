package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@Configurable
@TeleOp
public class DriveOnlyTeleop extends OpMode {

    private Follower follower;
    public static Pose startingPose;
    private TelemetryManager telemetryM;

    private Pose holdPose;
    private boolean holding;

    private final double deadzone = 3;

    private double kTrans = 0.1;
    private double kHeading = 1.4;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        double f = gamepad1.left_stick_y;
        double s = gamepad1.left_stick_x;
        double t = gamepad1.right_stick_x;

        boolean idle =
                Math.abs(f) < deadzone &&
                        Math.abs(s) < deadzone &&
                        Math.abs(t) < deadzone;

        Pose current = follower.getPose();

        if (idle) {
            if (!holding) {
                holdPose = current;
                holding = true;
            }

            double ex = holdPose.getX() - current.getX();
            double ey = holdPose.getY() - current.getY();

            double heading = current.getHeading();

            double cos = Math.cos(heading);
            double sin = Math.sin(heading);

            double robotX =  ex * cos + ey * sin;
            double robotY = -ex * sin + ey * cos;

            double eh = holdPose.getHeading() - heading;

            follower.setTeleOpDrive(
                    -robotX * kTrans,
                    -robotY * kTrans,
                    -eh * kHeading,
                    true
            );
        }
        else {
            holding = false;

            follower.setTeleOpDrive(
                    f,
                    s,
                    t,
                    true
            );
        }

        telemetryM.debug("pose", current);
        telemetryM.debug("holding", holding);
    }
}
