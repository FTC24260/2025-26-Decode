package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.ArtifactPipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.SpindexSubsystem;

@TeleOp(name = "FullTeleOp")
public class Teleop extends OpMode {

    private DriveSubsystem driveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private SpindexSubsystem spindexSubsystem;
    private TelemetryManager telemetryM;

    private VisionPortal visionPortal;
    private ArtifactPipeline pipeline;
    private DcMotor intake;

    public static double targetVelocity = 100;

    private long intakeBurstUntil = 0;
    private static final long INTAKE_BURST_MS = 800;

    private enum RapidFireState {
        IDLE,
        SPINUP_WAIT,
        FLICK_UP
    }

    private RapidFireState rapidFireState = RapidFireState.IDLE;
    private int rapidFireIndex = 0;
    private long rapidFireTimer = 0;

    private static final long SPINUP_DELAY_MS = 1000;
    private static final long FLICK_UP_MS = 200;

    private boolean lastA = false;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        spindexSubsystem = new SpindexSubsystem(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");

        pipeline = new ArtifactPipeline();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(pipeline)
                .build();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        double y = -gamepad2.left_stick_y;
        double x = gamepad2.left_stick_x;
        double rx = gamepad2.right_stick_x;

        driveSubsystem.teleopDrive(y, x, rx);
        driveSubsystem.update();

        if (now < intakeBurstUntil) {
            intake.setPower(-1);
        } else {
            intake.setPower(pipeline.ballDetected ? -1 : 0);
        }

        spindexSubsystem.updateSlots();

        if (rapidFireState != RapidFireState.IDLE) {
            shooterSubsystem.setTargetVelocity(targetVelocity);
        }

        switch (rapidFireState) {
            case SPINUP_WAIT:
                spindexSubsystem.setShootPosition(rapidFireIndex);
                if (now >= rapidFireTimer) {
                    spindexSubsystem.flickUp();
                    rapidFireTimer = now + FLICK_UP_MS;
                    rapidFireState = RapidFireState.FLICK_UP;
                }
                break;

            case FLICK_UP:
                if (now >= rapidFireTimer) {
                    spindexSubsystem.flickDown();
                    spindexSubsystem.resetSlots();

                    if (rapidFireIndex < 2 && spindexSubsystem.anySlotLoaded()) {
                        rapidFireIndex++;
                        intakeBurstUntil = now + INTAKE_BURST_MS;
                        rapidFireTimer = now + SPINUP_DELAY_MS;
                        rapidFireState = RapidFireState.SPINUP_WAIT;
                    } else {
                        rapidFireState = RapidFireState.IDLE;
                        shooterSubsystem.stopShooter();
                        rapidFireIndex = 0;
                    }
                }
                break;
        }

        if (gamepad1.a && !lastA && spindexSubsystem.anySlotLoaded()) {
            intakeBurstUntil = now + INTAKE_BURST_MS;
            rapidFireIndex = 0;
            rapidFireTimer = now + SPINUP_DELAY_MS;
            rapidFireState = RapidFireState.SPINUP_WAIT;
        }
        lastA = gamepad1.a;

        telemetry.addData("Slots", String.join(", ", spindexSubsystem.getSlots()));
        telemetry.addData("RapidFire", rapidFireState);
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0);
        shooterSubsystem.stopShooter();
        visionPortal.close();
    }
}
