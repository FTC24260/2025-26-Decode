package org.firstinspires.ftc.teamcode.pedroPathing.Vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ArtifactPipeline implements VisionProcessor {

    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();

    public boolean ballDetected = false;
    public double largestArea = 0;

    // Stricter HSV thresholds for colored ball detection
    private final double MIN_SAT = 100;  // ignore low-saturation (grayish) areas
    private final double MIN_VAL = 80;   // ignore dark areas
    private final double MAX_VAL = 200;  // ignore very bright areas (white)

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // no initialization needed
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Create mask for colored pixels
        Core.inRange(hsv,
                new Scalar(0, MIN_SAT, MIN_VAL),
                new Scalar(179, 255, MAX_VAL),
                mask);

        Imgproc.erode(mask, mask, new Mat());
        Imgproc.dilate(mask, mask, new Mat());

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        ballDetected = false;
        largestArea = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            if (area > largestArea) largestArea = area;

            // Only detect reasonably big colored blobs
            if (area > 1000) { // increased from 500 to 1000
                ballDetected = true;
                Imgproc.drawContours(input, List.of(contour), -1, new Scalar(255, 0, 0), 2); // blue overlay
            }
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity,
                            Object userContext) {
        // optional overlay
    }
}
