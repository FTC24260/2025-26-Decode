package org.firstinspires.ftc.teamcode.pedroPathing.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Green Artifact Detection")
public class GreenArtifactDetection extends LinearOpMode {

    private VisionPortal visionPortal;
    private GreenArtifactPipeline pipeline;

    private DcMotor intake;  // intake motor

    @Override
    public void runOpMode() {

        pipeline = new GreenArtifactPipeline();

        intake = hardwareMap.get(DcMotor.class, "intake"); // make sure name matches config
        intake.setPower(0);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(pipeline)
                .build();

        telemetry.addLine("Camera initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (pipeline.greenDetected) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            telemetry.addData("Green Detected", pipeline.greenDetected);
            telemetry.addData("Largest Area", pipeline.largestArea);
            telemetry.update();
        }

        visionPortal.close();
    }
}
