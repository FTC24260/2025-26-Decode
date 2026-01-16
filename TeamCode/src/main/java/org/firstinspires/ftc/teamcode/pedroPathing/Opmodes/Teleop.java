package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.pedroPathing.Commands.RapidFireCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.SpindexSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.ArtifactPipeline;

@TeleOp(name = "FullTeleOp")
public class Teleop extends OpMode {

    private DriveSubsystem driveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private SpindexSubsystem spindexSubsystem;
    private TurretSubsystem turretSubsystem;
    private RapidFireCommand rapidFire;
    private TelemetryManager telemetryM;

    private VisionPortal visionPortal;
    private ArtifactPipeline pipeline;
    private DcMotor intake;

    public static double targetVelocity = 100;

    private long intakeBurstUntil = 0;
    private static final long INTAKE_BURST_MS = 800;

    private boolean lastA = false;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        spindexSubsystem = new SpindexSubsystem(hardwareMap);
        turretSubsystem = new TurretSubsystem(hardwareMap);
        rapidFire = new RapidFireCommand(shooterSubsystem, spindexSubsystem);

        intake = hardwareMap.get(DcMotor.class, "intake");

        pipeline = new ArtifactPipeline();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(pipeline)
                .build();
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        double y = -gamepad2.left_stick_y;
        double x = gamepad2.left_stick_x;
        double rx = gamepad2.right_stick_x;

        driveSubsystem.teleopDrive(y, x, rx);
        driveSubsystem.update();

        turretSubsystem.update();

        if (now < intakeBurstUntil) {
            intake.setPower(-1);
        } else {
            intake.setPower(pipeline.ballDetected ? -1 : 0);
        }

        spindexSubsystem.updateSlots();

        if (gamepad1.a && !lastA && !rapidFire.isActive()) {
            intakeBurstUntil = now + INTAKE_BURST_MS;
            rapidFire.start(targetVelocity);
        }
        lastA = gamepad1.a;

        rapidFire.update();

        telemetry.addData("Slots", String.join(", ", spindexSubsystem.getSlots()));
        telemetry.addData("RapidFire", rapidFire.isActive());
        telemetry.addData("Turret Locked", turretSubsystem.hasTarget());
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0);
        shooterSubsystem.stopShooter();
        turretSubsystem.stop();
        visionPortal.close();
    }
}
