package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;

@TeleOp(name = "Turret Hold Forward Pinpoint", group = "Pedro Pathing")
public class TurretTest extends OpMode {

    private Follower follower;
    private DcMotor turret;

    private final int TURRET_MAX = 430;
    private final int TURRET_MIN = -530;
    private final double MAX_POWER = 0.6; // small power to hold position
    private final double Kp = 0.02;

    private int turretZero = 0;       // tick 0 = forward
    private double headingZero = 0;   // robot heading at init

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        // Initialize robot “forward” as 90° in radians
        follower.setStartingPose(new Pose(0, 0, 0));

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setDirection(DcMotor.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Treat current turret position as tick 0 (forward)
        turretZero = turret.getCurrentPosition();
        headingZero = follower.getPoseTracker().getPose().getHeading();
    }

    @Override
    public void loop() {
        follower.update();

        double currentHeading = follower.getPoseTracker().getPose().getHeading();

        // Compute how much the robot has rotated since init
        double deltaHeading = currentHeading - headingZero;

        // Convert deltaHeading to turret ticks (negate rotation)
        double ticksPerRadian = (TURRET_MAX - TURRET_MIN) / (2 * Math.PI);
        int turretOffset = (int)(-deltaHeading * ticksPerRadian);

        // Wrap turret offset if needed
        if (turretOffset > TURRET_MAX) turretOffset = TURRET_MIN + (turretOffset - TURRET_MAX - 1);
        if (turretOffset < TURRET_MIN) turretOffset = TURRET_MAX - (TURRET_MIN - turretOffset - 1);

        // Target = tick 0 + offset
        int target = turretZero + turretOffset;

        // Proportional control
        int error = target - turret.getCurrentPosition();
        double power = Kp * error;
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        turret.setPower(power);

        telemetry.addData("Robot Heading (deg)", Math.toDegrees(currentHeading));
        telemetry.addData("Turret Position", turret.getCurrentPosition());
        telemetry.addData("Turret Target", target);
        telemetry.addData("Turret Error", error);
        telemetry.addData("Turret Power", power);
        telemetry.update();
    }
}
