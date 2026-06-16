package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

/**
 * Pinpoint Sensor Debug Test OpMode
 *
 * This OpMode helps diagnose if your Pinpoint localizer is working correctly.
 * It displays:
 * - Real-time X, Y, Heading from Pinpoint
 * - Raw encoder values
 * - Pinpoint status and device info
 * - Updates in real-time as the robot moves
 *
 * Steps to use:
 * 1. Place robot at field origin (0, 0) facing angle 0
 * 2. Push INIT to start
 * 3. Watch telemetry - X, Y, Heading should stay near (0, 0, 0)
 * 4. Manually push the robot forward/back and left/right
 * 5. Check if X, Y, Heading update correctly
 * 6. If values don't change or are erratic, Pinpoint may be failing
 *
 */
@TeleOp(name = "Pinpoint Debug Test")
public class PinpointDebugTest extends OpMode {

    private Follower follower;
    private GoBildaPinpointDriver pinpoint;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    private double lastX = 0, lastY = 0, lastHeading = 0;
    private long lastUpdateTime = 0;

    @Override
    public void init() {
        telemetry.addLine("Initializing Pinpoint Debug Test...");
        telemetry.update();

        // Create follower (which initializes Pinpoint)
        follower = Constants.createFollower(hardwareMap);

        // Try to get pinpoint device directly for additional debug info
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            telemetry.addLine("✓ Pinpoint device found!");
        } catch (Exception e) {
            telemetry.addLine("✗ Pinpoint device NOT found in hardwareMap!");
            telemetry.addData("Error", e.getMessage());
        }

        // Get drive motors to display encoder values
        try {
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
            telemetry.addLine("✓ Drive motors found!");
        } catch (Exception e) {
            telemetry.addLine("✗ Could not find all drive motors");
        }

        // Set starting pose
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();

        telemetry.addLine("Initialization complete!");
        telemetry.addLine("");
        telemetry.addLine("INSTRUCTIONS:");
        telemetry.addLine("1. Place robot at origin facing 0°");
        telemetry.addLine("2. Push START to begin");
        telemetry.addLine("3. Manually move the robot and watch telemetry");
        telemetry.addLine("4. Values should update smoothly");
        telemetry.update();

        lastUpdateTime = System.currentTimeMillis();
    }

    @Override
    public void start() {
        telemetry.addLine("Test started! Move the robot...");
        telemetry.update();
        lastUpdateTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        // Update follower (this updates Pinpoint)
        follower.update();

        long now = System.currentTimeMillis();
        long deltaTime = now - lastUpdateTime;

        Pose currentPose = follower.getPose();
        double deltaX = currentPose.getX() - lastX;
        double deltaY = currentPose.getY() - lastY;
        double deltaHeading = currentPose.getHeading() - lastHeading;

        // Normalize heading delta to -π to π
        while (deltaHeading > Math.PI) deltaHeading -= 2 * Math.PI;
        while (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;

        telemetry.addLine("========== PINPOINT POSITION ==========");
        telemetry.addData("X (mm)", "%.2f", currentPose.getX());
        telemetry.addData("Y (mm)", "%.2f", currentPose.getY());
        telemetry.addData("Heading (rad)", "%.4f", currentPose.getHeading());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(currentPose.getHeading()));

        telemetry.addLine("");
        telemetry.addLine("========== DELTA (CHANGE) ==========");
        telemetry.addData("ΔX (mm)", "%.2f", deltaX);
        telemetry.addData("ΔY (mm)", "%.2f", deltaY);
        telemetry.addData("ΔHeading (deg)", "%.2f", Math.toDegrees(deltaHeading));
        telemetry.addData("Time elapsed (ms)", deltaTime);

        telemetry.addLine("");
        telemetry.addLine("========== VELOCITY ==========");
        if (deltaTime > 0) {
            telemetry.addData("X velocity (mm/s)", "%.2f", deltaX / deltaTime * 1000);
            telemetry.addData("Y velocity (mm/s)", "%.2f", deltaY / deltaTime * 1000);
        }

        telemetry.addLine("");
        telemetry.addLine("========== MOTOR ENCODERS ==========");
        if (leftFront != null) {
            telemetry.addData("Left Front", leftFront.getCurrentPosition());
        }
        if (leftRear != null) {
            telemetry.addData("Left Rear", leftRear.getCurrentPosition());
        }
        if (rightFront != null) {
            telemetry.addData("Right Front", rightFront.getCurrentPosition());
        }
        if (rightRear != null) {
            telemetry.addData("Right Rear", rightRear.getCurrentPosition());
        }

        telemetry.addLine("");
        telemetry.addLine("========== PINPOINT STATUS ==========");
        if (pinpoint != null) {
            telemetry.addData("Pinpoint Device Status", "Connected");
            try {
                telemetry.addData("Device Version", pinpoint.getDeviceVersion());
                telemetry.addData("IntoX Offset (in)", "%.2f", pinpoint.getXOffset(DistanceUnit.INCH));
                telemetry.addData("IntoY Offset (in)", "%.2f", pinpoint.getYOffset(DistanceUnit.INCH));
            } catch (Exception e) {
                telemetry.addData("Status Query Error", e.getMessage());
            }
        } else {
            telemetry.addData("Pinpoint Device Status", "NOT FOUND!");
        }

        telemetry.addLine("");
        telemetry.addLine("========== DEBUG INFO ==========");
        telemetry.addLine("If X/Y/Heading values don't update when you:");
        telemetry.addLine("- Move the robot → Pinpoint may be frozen");
        telemetry.addLine("- Rotate the robot → Heading encoder may be broken");
        telemetry.addLine("- See jumps/spikes → Improper wheel contact/slipping");

        lastX = currentPose.getX();
        lastY = currentPose.getY();
        lastHeading = currentPose.getHeading();
        lastUpdateTime = now;

        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addLine("Test stopped");
        telemetry.update();
    }
}
