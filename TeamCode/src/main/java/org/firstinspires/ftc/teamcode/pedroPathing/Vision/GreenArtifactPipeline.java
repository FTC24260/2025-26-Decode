package org.firstinspires.ftc.teamcode.pedroPathing.Vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class GreenArtifactPipeline implements VisionProcessor {

    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();

    public boolean greenDetected = false;
    public double largestArea = 0;

    private final Scalar LOWER_GREEN = new Scalar(35, 80, 80);
    private final Scalar UPPER_GREEN = new Scalar(85, 255, 255);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // No init needed
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, LOWER_GREEN, UPPER_GREEN, mask);

        Imgproc.erode(mask, mask, new Mat());
        Imgproc.dilate(mask, mask, new Mat());

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        greenDetected = false;
        largestArea = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            if (area > largestArea) {
                largestArea = area;
            }

            if (area > 500) {
                greenDetected = true;
                Imgproc.drawContours(
                        input,
                        List.of(contour),
                        -1,
                        new Scalar(0, 255, 0),
                        2
                );
            }
        }

        return null; // VisionPortal ignores this
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity,
                            Object userContext) {
        // Optional overlay
    }
}
