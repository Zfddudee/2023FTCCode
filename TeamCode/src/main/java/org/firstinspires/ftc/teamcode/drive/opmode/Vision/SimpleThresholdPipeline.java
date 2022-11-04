package org.firstinspires.ftc.teamcode.drive.opmode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SimpleThresholdPipeline extends OpenCvPipeline {
    public Scalar lowerScalar = new Scalar(110,50,50);
    public Scalar upperSclar = new Scalar(130,255,255);

    public boolean showGreen = true;
    public boolean showPurlple = false;
    public boolean showOrange = false;

    private Telemetry telemetry;

    public SimpleThresholdPipeline(Telemetry tela){
        telemetry = tela;
    }
    @Override
    public Mat processFrame(Mat input) {

        if(showGreen){
            lowerScalar = new Scalar(40,50,50);
            upperSclar = new Scalar(80,255,255);
        }
        else if(showOrange){
            lowerScalar = new Scalar(100,50,255);
            upperSclar = new Scalar(140,255,255);
        }
        else if(showPurlple){
            lowerScalar = new Scalar(130,50,50);
            upperSclar = new Scalar(170,255,255);
        }


        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_BGR2HSV);

        Mat mask = new Mat();
        Core.inRange(hsvMat, lowerScalar, upperSclar, mask);

        //remove noise
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, new Mat());
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, new Mat());

        List<MatOfPoint> contourPoints = new ArrayList<>();
        Mat hierarchyMat = new Mat();
        Imgproc.findContours(mask, contourPoints, hierarchyMat, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        telemetry.addData("Contour Points: ", contourPoints.size());

        /*
         * Release the reusable Mat so that old data doesn't
         * affect the next step in the current processing
         */
        hsvMat.release();
        //greenMask.release();

        //Core.bitwise_and(input, input, maskedInputMat, binaryMat);

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
        return mask;
    }

}