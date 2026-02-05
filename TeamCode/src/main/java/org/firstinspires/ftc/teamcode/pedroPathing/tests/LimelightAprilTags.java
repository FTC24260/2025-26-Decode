package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Autonomous(name = "Limelight Multi-Pipeline Detection", group = "Vision")
public class LimelightAprilTags extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize limelight - make sure "limelight" matches your hardware config name
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        // Start the limelight before waitForStart() for initializations
        limelight.start();

        telemetry.addData("Status", "Initialized - Ready to test pipelines");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Clear previous telemetry
            telemetry.clear();

            // Always show we're in the loop
            telemetry.addData("Loop Status", "Running...");

            boolean foundInPipeline0 = false;
            boolean foundInPipeline1 = false;
            boolean foundInPipeline2 = false;

            // Check Pipeline 0
            limelight.pipelineSwitch(0);
            sleep(50); // Give time for pipeline switch
            LLResult result0 = limelight.getLatestResult();

            if (result0 != null && result0.isValid()) {
                foundInPipeline0 = true;
                telemetry.addData("Pipeline 0", "AprilTag detected!");
            } else {
                telemetry.addData("Pipeline 0", "No detection");
            }

            // Check Pipeline 1 only if not found in Pipeline 0
            if (!foundInPipeline0) {
                limelight.pipelineSwitch(1);
                sleep(50); // Give time for pipeline switch
                LLResult result1 = limelight.getLatestResult();

                if (result1 != null && result1.isValid()) {
                    foundInPipeline1 = true;
                    telemetry.addData("Pipeline 1", "AprilTag detected!");
                } else {
                    telemetry.addData("Pipeline 1", "No detection");
                }
            }

            // Check Pipeline 2 only if not found in Pipeline 0 or 1
            if (!foundInPipeline0 && !foundInPipeline1) {
                limelight.pipelineSwitch(2);
                sleep(50); // Give time for pipeline switch
                LLResult result2 = limelight.getLatestResult();

                if (result2 != null && result2.isValid()) {
                    foundInPipeline2 = true;
                    telemetry.addData("Pipeline 2", "AprilTag detected!");
                } else {
                    telemetry.addData("Pipeline 2", "No detection");
                }
            }

            // Determine the result based on detections
            if (foundInPipeline0) {
                telemetry.addData("Result", "GREEN - PURPLE - PURPLE");
            } else if (foundInPipeline1) {
                telemetry.addData("Result", "PURPLE - GREEN - PURPLE");
            } else if (foundInPipeline2) {
                telemetry.addData("Result", "PURPLE - PURPLE - GREEN");
            } else {
                telemetry.addData("Result", "NO APRILTAG DETECTED IN ANY PIPELINE");
            }

            // MUST have this to see updates
            telemetry.update();

            sleep(200); // Slow it down for easier reading and pipeline switching
        }

        // Stop the limelight when done
        limelight.stop();
    }
}
