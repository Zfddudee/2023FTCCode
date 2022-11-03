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

    public static Scalar DISPLAY_COLOR = new Scalar(210, 150, 190);
    public Scalar lowerGreen = new Scalar(0, 72, 0);
    public Scalar lowerBlue = new Scalar(0, 72, 0);
    public Scalar lowerRed = new Scalar(0, 72, 0);

    public Scalar upperGreen = new Scalar(255, 93, 255);
    public Scalar upperBlue = new Scalar(124, 104, 215);
    public Scalar upperRed = new Scalar(154, 255, 96);

    public int Last;
    public double areaRed;
    public double areaBlue;
    public double ErrorX = 0;
    public double ErrorY = 0;

    public static int BORDER_LEFT_X = 0;   //amount of pixels from the left side of the cam to skip
    public static int BORDER_RIGHT_X = 0;   //amount of pixels from the right of the cam to skip
    public static int BORDER_TOP_Y = 70;   //amount of pixels from the top of the cam to skip
    public static int BORDER_BOTTOM_Y = 70;   //amount of pixels from the bottom of the cam to skip

    public static double MIN_AREA = 500;
    private double maxArea = 0;


    public Exception debug;
    public double xRed = -1;
    public double yRed = -1;
    public double xBlue = -1;
    public double yBlue = -1;


//added
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
    private Mat MaskBlue = new Mat();
    private Mat MaskRed = new Mat();
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
            Imgproc.cvtColor(input, ycrcbMat, colorSpace.cvtCode);
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
            Core.inRange(ycrcbMat, lowerRed, upperRed, MaskRed);
            Core.inRange(ycrcbMat, lowerBlue, upperBlue , MaskBlue);

            // Remove Noise
            //Green
            Imgproc.morphologyEx(MaskGreen, MaskGreen, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(MaskGreen, MaskGreen, Imgproc.MORPH_CLOSE, new Mat());
            //Red
            Imgproc.morphologyEx(MaskRed, MaskRed, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(MaskRed, MaskRed, Imgproc.MORPH_CLOSE, new Mat());
            //Blue
            Imgproc.morphologyEx(MaskBlue, MaskBlue, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(MaskBlue, MaskBlue, Imgproc.MORPH_CLOSE, new Mat());

            // GaussianBlur
            //Green
            Imgproc.GaussianBlur(MaskGreen, MaskGreen, new Size(5.0, 5.0), 0.00);
            List<MatOfPoint> contoursGreen = new ArrayList<>();
            //Red
            Imgproc.GaussianBlur(MaskRed, MaskRed, new Size(5.0, 5.0), 0.00);
            List<MatOfPoint> contoursRed = new ArrayList<>();
            //Blue
            Imgproc.GaussianBlur(MaskBlue, MaskBlue, new Size(5.0, 5.0), 0.00);
            List<MatOfPoint> contoursBlue = new ArrayList<>();

            //finding the contours
            //Green
            Mat hierarchyGreen = new Mat();
            Imgproc.findContours(MaskGreen, contoursGreen, hierarchyGreen, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(MaskGreen, contoursGreen, -1, DISPLAY_COLOR, 2);
            //Red
            Mat hierarchyRed = new Mat();
            Imgproc.findContours(MaskRed, contoursRed, hierarchyRed, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(MaskRed, contoursRed, -1, DISPLAY_COLOR, 2);
            //Blue
            Mat hierarchyBlue = new Mat();
            Imgproc.findContours(MaskBlue, contoursBlue, hierarchyBlue, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(MaskBlue, contoursBlue, -1, DISPLAY_COLOR, 2);


            // finding the coordinates of red contour.
            List<Moments> muRed = new ArrayList<Moments>(contoursRed.size());
            for (int i = 0; i < contoursRed.size(); i++) {
                muRed.add(i, Imgproc.moments(contoursRed.get(i), false));
                Moments mRed = muRed.get(i);
                xRed = -1;
                xRed = (int) (mRed.get_m10() / mRed.get_m00());
                yRed = (int) (mRed.get_m01() / mRed.get_m00());
                Imgproc.circle(MaskRed, new Point(xRed, yRed), 4, new Scalar(255, 49, 0, 255));
            }
            for (MatOfPoint contourRed : contoursRed) {
                Point[] contourArrayRed = contourRed.toArray();
                MatOfPoint2f areaPointsRed = new MatOfPoint2f(contourArrayRed);
                Rect rectRed = Imgproc.boundingRect(areaPointsRed);
                areaRed = rectRed.area();
                Imgproc.rectangle(MaskRed, rectRed, DISPLAY_COLOR, 2);
            }

            // finding the coordinates of blue contour.
            List<Moments> muBlue = new ArrayList<Moments>(contoursBlue.size());
            for (int i = 0; i < contoursBlue.size(); i++) {
                muBlue.add(i, Imgproc.moments(contoursBlue.get(i), false));
                Moments mBlue = muBlue.get(i);
                xBlue = -1;
                xBlue = (int) (mBlue.get_m10() / mBlue.get_m00());
                yBlue = (int) (mBlue.get_m01() / mBlue.get_m00());
                Imgproc.circle(MaskBlue, new Point(xBlue, yBlue), 4, new Scalar(100, 100, 194, 255));
            }
            for (MatOfPoint contourBlue : contoursBlue) {
                Point[] contourArrayBlue = contourBlue.toArray();
                MatOfPoint2f areaPointsBlue = new MatOfPoint2f(contourArrayBlue);
                Rect rectBlue = Imgproc.boundingRect(areaPointsBlue);
                areaBlue = rectBlue.area();
                Imgproc.rectangle(MaskBlue, rectBlue, DISPLAY_COLOR, 2);
            }

            // setting last output value.
            if(contoursGreen.size() > 1) Last = 0;
            else if(contoursRed.size() > 1) Last = 1;
            else if(contoursBlue.size() > 1) Last = 2;

            //Imgproc.circle(binaryMat, RIGHT, 50, DISPLAY_COLOR);
            // Display Data
            /*
             * Release the reusable Mat so that old data doesn't
             * affect the next step in the current processing
             */
            //added

            //added


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
            Core.bitwise_and(input, input, maskedInputMat, MaskRed);
            Core.bitwise_and(input, input, maskedInputMat, MaskBlue);
            //Core.bitwise_and(input, input, maskedInputMat, MaskSides);
            /**
             * Add some nice and informative telemetry messages
             */
            telemetry.addData("[>]", "Change these values in tuner menu");
            telemetry.addData("[Color Space]", colorSpace.name());
            telemetry.addData("[Lower Scalar]", lowerGreen);
            telemetry.addData("[Upper Scalar]", upperGreen);
            telemetry.addData("[Position]", Last);
            telemetry.addData("xBlue", xBlue);
            telemetry.addData("yBlue", yBlue);
            telemetry.addData("xRed", xRed);
            telemetry.addData("yRed", yRed);
            telemetry.addData("SizeBlue", contoursBlue.size());
            telemetry.addData("SizeRed", contoursRed.size());
            telemetry.addData("SizeGreen", contoursGreen.size());
            telemetry.addData("area", areaRed);
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
}