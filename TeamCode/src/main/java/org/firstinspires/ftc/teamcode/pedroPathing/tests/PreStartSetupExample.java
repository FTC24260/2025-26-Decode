package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Decode PreStart Menu")
public class PreStartSetupExample extends LinearOpMode {

    int numBalls = 0;
    boolean preload = false;
    int spikeMark = 1; // options: 1, 2, 3
    boolean park = false;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing...");
        telemetry.update();
        sleep(1000);

        telemetry.addLine("Initialization Complete!");
        telemetry.update();

        // Pre-start menu loop
        while (!isStarted() && !isStopRequested()) {

            // --- Number of Balls ---
            if (gamepad1.dpad_up) numBalls++;
            if (gamepad1.dpad_down) numBalls--;
            if (numBalls < 0) numBalls = 0;

            // --- Preload ---
            if (gamepad1.a) preload = true;
            if (gamepad1.b) preload = false;

            // --- Spike Mark Selection ---
            if (gamepad1.x) spikeMark = 1;
            if (gamepad1.y) spikeMark = 2;
            if (gamepad1.b) spikeMark = 3; // alternate button for third option

            // --- Park Selection ---
            if (gamepad1.left_bumper) park = true;
            if (gamepad1.right_bumper) park = false;

            // --- Display selections ---
            telemetry.addLine("=== Autonomous Setup ===");
            telemetry.addData("Number of Balls (Dpad Up/Down):", numBalls);
            telemetry.addData("Preload (A=True, B=False):", preload);
            telemetry.addData("Spike Mark (X=1, Y=2, B=3):", spikeMark);
            telemetry.addData("Park? (LB=True, RB=False):", park);
            telemetry.addLine("Press START when ready!");
            telemetry.update();

            sleep(200); // small delay to prevent flooding
        }

        telemetry.clearAll();
        telemetry.addLine("Starting Autonomous with selections:");
        telemetry.addData("Number of Balls:", numBalls);
        telemetry.addData("Preload:", preload);
        telemetry.addData("Spike Mark:", spikeMark);
        telemetry.addData("Park:", park);
        telemetry.update();

        // --- Autonomous path logic example ---
        if (preload) {
            // Run preload routine
        }

        for (int i = 0; i < numBalls; i++) {
            // Shoot or collect balls
        }

        // Go to spike mark
        switch (spikeMark) {
            case 1:
                // go to spike 1
                break;
            case 2:
                // go to spike 2
                break;
            case 3:
                // go to spike 3
                break;
        }

        if (park) {
            // Park in the designated area
        }
    }
}
