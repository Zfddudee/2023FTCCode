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
import java.util.List;

public class ImageDetectorPipelineArea extends OpenCvPipeline {

    public static Scalar DISPLAY_COLOR = new Scalar(210, 150, 190);
    public Scalar lowerGreen = new Scalar(40,50,50);;// = new Scalar(76,77,118);
    public Scalar upperGreen = new Scalar(80,255,255);;// = new Scalar(140, 123, 133);

    public int Last;

    public Exception debug;

    public ColorSpace colorSpace = ColorSpace.HSV;

    private Mat HSVMat = new Mat();
    private Mat MaskGreen = new Mat();
    private Mat maskedInputMat = new Mat();

    private Telemetry telemetry;

    enum ColorSpace {
        HSV(Imgproc.COLOR_RGB2HSV);

        public int cvtCode = 0;

        ColorSpace(int cvtCode) {
            this.cvtCode = cvtCode;
        }
    }

    public ImageDetectorPipelineArea(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        try {

            Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_RGB2HSV);

            Core.inRange(HSVMat, lowerGreen, upperGreen, MaskGreen);

            // Remove Noise
            Imgproc.morphologyEx(MaskGreen, MaskGreen, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(MaskGreen, MaskGreen, Imgproc.MORPH_CLOSE, new Mat());

            // GaussianBlur
            Imgproc.GaussianBlur(MaskGreen, MaskGreen, new Size(5.0, 5.0), 0.00);
            List<MatOfPoint> contours = new ArrayList<>();

            //finding the contours
            Mat hierarchy = new Mat();
            Imgproc.findContours(MaskGreen, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(MaskGreen, contours, -1, DISPLAY_COLOR, 2);

            double maxArea = 0;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                }
            }


            // setting last output value.
            if(maxArea > 3000)
                Last = 3;
            else if(maxArea < 3000 && maxArea > 1000)
                Last = 1;
            else
                Last = 2;


            maskedInputMat.release();

            Core.bitwise_and(input, input, maskedInputMat, MaskGreen);

            telemetry.addData("Area", maxArea);
            telemetry.update();

            return maskedInputMat;
        }
        catch (Exception e){
            debug = e;
            boolean error = true;
        }
        return MaskGreen;
    }


}