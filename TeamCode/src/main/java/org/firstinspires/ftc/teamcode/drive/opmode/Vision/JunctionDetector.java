/*
 * Copyright (c) 2021 Sebastian Erives
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

package org.firstinspires.ftc.teamcode.drive.opmode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Predicate;


public class JunctionDetector extends OpenCvPipeline {

    private Mat thresholdedImage;
    private Point centroid;
    private Telemetry telemetry;

    public double Thresh = 100;
    public double MaxVal = 255;
    public JunctionDetector() {
        thresholdedImage = new Mat();
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {

// Convert the image to grayscale
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);
// Apply thresholding to the image to create a binary image
        Imgproc.threshold(input, thresholdedImage, Thresh, MaxVal, Imgproc.THRESH_BINARY);

// Find the contours in the image
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresholdedImage, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

// Find the contour with the largest area (assumed to be the junction)
        MatOfPoint largestContour = null;
        double maxArea = 0;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

// Find the centroid of the largest contour
        Moments moments = Imgproc.moments(largestContour);
        centroid = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());

// Draw the centroid on the original image
        Imgproc.drawContours(input, Arrays.asList(largestContour), -1, new Scalar(255, 0, 0), -1);
        Imgproc.circle(input, centroid, 5, new Scalar(0, 0, 255), -1);

        if (telemetry != null) {
            telemetry.addData("X position of junction: ", centroid.x);
            telemetry.update();
        }

        return input;

    }
    public double getXPosition() {
        return centroid.x;
    }
}