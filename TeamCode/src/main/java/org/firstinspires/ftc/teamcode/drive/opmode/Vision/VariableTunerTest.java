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

public class VariableTunerTest extends OpenCvPipeline {

    /*
     * These are our variables that will be
     * modifiable from the variable tuner.
     *
     * Scalars in OpenCV are generally used to
     * represent color. So our values in the
     * lower and upper Scalars here represent
     * the Y, Cr and Cb values respectively.
     *
     * YCbCr, like most color spaces, range
     * from 0-255, so we default to those
     * min and max values here for now, meaning
     * that all pixels will be shown.
     */
    enum ColorIdentified
    {
        Green,
        Purple,
        Orange
    }

    public static Scalar DISPLAY_COLOR = new Scalar(210, 150, 190);
    public Scalar lowerGreen;// = new Scalar(76,77,118);
    public Scalar lowerPurple;// = new Scalar(0, 0, 140);
    public Scalar lowerOrange;// = new Scalar(0,0,93);   //28,85,90

    public Scalar upperGreen;// = new Scalar(140, 123, 133);
    public Scalar upperPurple;// = new Scalar(95, 163, 170);
    public Scalar upperOrange;// = new Scalar(255,255,120);

    public int Last;
    public int Sensitivity = 15;
    public ColorIdentified colorIdentified;
    public double areaOrange;
    public double areaPurple;
    public double areaGreen;
    //public double ErrorX = 0;
    //public double ErrorY = 0;

//    public static int BORDER_LEFT_X = 0;   //amount of pixels from the left side of the cam to skip
//    public static int BORDER_RIGHT_X = 0;   //amount of pixels from the right of the cam to skip
//    public static int BORDER_TOP_Y = 70;   //amount of pixels from the top of the cam to skip
//    public static int BORDER_BOTTOM_Y = 70;   //amount of pixels from the bottom of the cam to skip

//    public static double MIN_AREA = 500;
//    private double maxArea = 0;


    public Exception debug;
    public double xOrange = -1;
    public double yOrange = -1;
    public double xPurple = -1;
    public double yPurple = -1;
    public double xGreen = -1;
    public double yGreen = -1;

    /**
     * This will allow us to choose the color
     * space we want to use on the live field
     * tuner instead of hardcoding it
     */
    public ColorSpace colorSpace = ColorSpace.RGB;

    /*
     * A good practice when typing EOCV pipelines is
     * declaring the Mats you will use here at the top
     * of your pipeline, to reuse the same buffers every
     * time. This removes the need to call mat.release()
     * with every Mat you create on the processFrame method,
     * and therefore, reducing the possibility of getting a
     * memory leak and causing the app to crash due to an
     * "Out of Memory" error.
     */
    private Mat ycrcbMat = new Mat();
    private Mat MaskGreen = new Mat();
    private Mat MaskPurple = new Mat();
    private Mat MaskOrange = new Mat();
    //private Mat MaskSides = new Mat();
    private Mat maskedInputMat = new Mat();

    private Telemetry telemetry;

    /**
     * Enum to choose which color space to choose
     * with the live variable tuner isntead of
     * hardcoding it.
     */
    enum ColorSpace {
        /*
         * Define our "conversion codes" in the enum
         * so that we don't have to do a switch
         * statement in the processFrame method.
         */
        RGB(Imgproc.COLOR_RGBA2RGB);
        //HSV(Imgproc.COLOR_RGB2HSV),
        //YCrCb(Imgproc.COLOR_RGB2YCrCb),
        //Lab(Imgproc.COLOR_RGB2Lab);
        //GRAY(Imgproc.COLOR_RGB2GRAY)

        //store cvtCode in a public var
        public int cvtCode = 0;

        //constructor to be used by enum declarations above
        ColorSpace(int cvtCode) {
            this.cvtCode = cvtCode;
        }
    }

    public VariableTunerTest(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        try {
            // Green: (144,77,65)
            int g1 = 40, g2 = 50, g3 = 50;
            lowerGreen = new Scalar(40,50,50);
            upperGreen = new Scalar(80,255,255);

            //orange  (28,85,90)
            int o1 = 28, o2 = 85, o3 = 90;
            lowerOrange = new Scalar(100,50,255);   //28,85,90
            upperOrange = new Scalar(140,255,255);

            //purple:  (270,66,60)
            int p1 = 270, p2 = 66, p3 = 60;
            lowerPurple = new Scalar(130,50,50);
            upperPurple = new Scalar(170,255,255);


            /*
             * Converts our input mat from RGB to
             * specified color space by the enum.
             * EOCV ALWAYS returns RGB mats, so you'd
             * always convert from RGB to the color
             * space you want to use.
             *
             * Takes our "input" mat as an input, and outputs
             * to a separate Mat buffer "ycrcbMat"
             */
            //Imgproc.cvtColor(input, ycrcbMat, colorSpace.cvtCode);
            //Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_BGR2GRAY);
            Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

            /*
             * This is where our thresholding actually happens.
             * Takes our "ycrcbMat" as input and outputs a "binary"
             * Mat to "binaryMat" of the same size as our input.
             * "Discards" all the pixels outside the bounds specified
             * by the scalars above (and modifiable with EOCV-Sim's
             * live variable tuner.)
             *
             * Binary meaning that we have either a 0 or 255 value
             * for every pixel.
             *
             * 0 represents our pixels that were outside the bounds
             * 255 represents our pixels that are inside the bounds
             */
            Core.inRange(ycrcbMat, lowerGreen, upperGreen, MaskGreen);
            Core.inRange(ycrcbMat, lowerOrange, upperOrange, MaskOrange);
            Core.inRange(ycrcbMat, lowerPurple, upperPurple, MaskPurple);

            // Remove Noise
            //Green
            Imgproc.morphologyEx(MaskGreen, MaskGreen, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(MaskGreen, MaskGreen, Imgproc.MORPH_CLOSE, new Mat());
            //Red
            Imgproc.morphologyEx(MaskOrange, MaskOrange, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(MaskOrange, MaskOrange, Imgproc.MORPH_CLOSE, new Mat());
            //Blue
            Imgproc.morphologyEx(MaskPurple, MaskPurple, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(MaskPurple, MaskPurple, Imgproc.MORPH_CLOSE, new Mat());

            // GaussianBlur
            //Green
            Imgproc.GaussianBlur(MaskGreen, MaskGreen, new Size(5.0, 5.0), 0.00);
            List<MatOfPoint> contoursGreen = new ArrayList<>();
            //Orange
            Imgproc.GaussianBlur(MaskOrange, MaskOrange, new Size(5.0, 5.0), 0.00);
            List<MatOfPoint> contoursOrange = new ArrayList<>();
            //Purple
            Imgproc.GaussianBlur(MaskPurple, MaskPurple, new Size(5.0, 5.0), 0.00);
            List<MatOfPoint> contoursPurple = new ArrayList<>();

            //finding the contours
            //Green
            Mat hierarchyGreen = new Mat();
            ImgProcAssignment(MaskGreen, contoursGreen, hierarchyGreen);
            //Orange
            Mat hierarchyOrange = new Mat();
            ImgProcAssignment(MaskOrange, contoursOrange, hierarchyOrange);
            //Purple
            Mat hierarchyPurple = new Mat();
            ImgProcAssignment(MaskPurple, contoursPurple, hierarchyPurple);

            // finding the coordinates of purple contour.
            FindColor(contoursPurple, xPurple, yPurple, areaPurple, MaskPurple);

            // finding the coordinates of orange contour.
            FindColor(contoursOrange, xOrange, yOrange, areaOrange, MaskOrange);

            // finding the coordinates of green contour.
            FindColor(contoursGreen, xGreen, yGreen, areaGreen, MaskGreen);

            // setting last output value.
            if(contoursGreen.size() > 1) {
                Last = 0;
                colorIdentified = ColorIdentified.Green;
            }
            else if(contoursOrange.size() > 1) {
                Last = 1;
                colorIdentified = ColorIdentified.Orange;
            }
            else if(contoursPurple.size() > 1) {
                Last = 2;
                colorIdentified = ColorIdentified.Purple;
            }

            maskedInputMat.release();

            /*
             * Now, with our binary Mat, we perform a "bitwise and"
             * to our input image, meaning that we will perform a mask
             * which will include the pixels from our input Mat which
             * are "255" in our binary Mat (meaning that they're inside
             * the range) and will discard any other pixel outside the
             * range (RGB 0, 0, 0. All discarded pixels will be black)
             */
            Core.bitwise_and(input, input, maskedInputMat, MaskGreen);
            Core.bitwise_and(input, input, maskedInputMat, MaskOrange);
            Core.bitwise_and(input, input, maskedInputMat, MaskPurple);
            //Core.bitwise_and(input, input, maskedInputMat, MaskSides);
            /**
             * Add some nice and informative telemetry messages
             */
            //telemetry.addData("[>]", "Change these values in tuner menu");
//            telemetry.addData("[Color Space]", colorSpace.name());
//            telemetry.addData("[Lower Scalar]", lowerGreen);
//            telemetry.addData("[Upper Scalar]", upperGreen);
//            telemetry.addData("[Position]", Last);
//            telemetry.addData("xBlue", xPurple);
//            telemetry.addData("yBlue", yPurple);
//            telemetry.addData("xOrange", xOrange);
//            telemetry.addData("yOrange", yOrange);
//            telemetry.addData("SizeBlue", contoursPurple.size());
//            telemetry.addData("SizeRed", contoursOrange.size());
//            telemetry.addData("SizeGreen", contoursGreen.size());
//            telemetry.addData("areaOrange", areaOrange);
            telemetry.addData("Color Identified", colorIdentified);
            telemetry.update();

            /*
             * The Mat returned from this method is the
             * one displayed on the viewport.
             *
             * To visualize our threshold, we'll return
             * the "masked input mat" which shows the
             * pixel from the input Mat that were inside
             * the threshold range.
             */
            return maskedInputMat;
        }
        //catch kinda like a else statement where if the try fails it goes to the catch.
        catch (Exception e){
            debug = e;
            boolean error = true;
        }
        return MaskGreen;
    }

    private void FindColor(List<MatOfPoint> contoursGreen, double xGreen, double yGreen, double areaGreen, Mat maskGreen) {
        List<Moments> muGreen = new ArrayList<Moments>(contoursGreen.size());
        for (int i = 0; i < contoursGreen.size(); i++) {
            muGreen.add(i, Imgproc.moments(contoursGreen.get(i), false));
            Moments mGreen = muGreen.get(i);
            xGreen = -1;
            xGreen = (int) (mGreen.get_m10() / mGreen.get_m00());
            yGreen = (int) (mGreen.get_m01() / mGreen.get_m00());
            //Imgproc.circle(MaskGreen, new Point(xGreen, yGreen), 4, new Scalar(100, 100, 194, 255));
            //Imgproc.circle(MaskGreen, new Point(xGreen, yGreen), 4, upperGreen);
        }
        for (MatOfPoint contourGreen : contoursGreen) {
            Point[] contourArrayGreen = contourGreen.toArray();
            MatOfPoint2f areaPointsGreen = new MatOfPoint2f(contourArrayGreen);
            Rect rectGreen = Imgproc.boundingRect(areaPointsGreen);
            areaGreen = rectGreen.area();
            Imgproc.rectangle(maskGreen, rectGreen, DISPLAY_COLOR, 2);
        }
    }

    private void ImgProcAssignment(Mat mat, List<MatOfPoint> contours, Mat hierarchy) {
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(mat, contours, -1, DISPLAY_COLOR, 10);
    }
}