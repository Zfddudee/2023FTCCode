package org.firstinspires.ftc.teamcode.drive.opmode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
public class ImageDetectorPipeline extends OpenCvPipeline {
    public Scalar lScalar = new Scalar(110,50,50);
    public Scalar uScalar = new Scalar(130,255,255);

    public boolean showGreen = true;
    public boolean showPurlple = true;
    public boolean showOrange = true;

    private ArrayList<ColorRange> colors = new ArrayList<>();
    final int AREA_THRESHOLD = 10000;

    private Telemetry telemetry;

    public String ColorSeen = "";

    public ImageDetectorPipeline(Telemetry tela){
        telemetry = tela;
        if(showGreen){
            lScalar = new Scalar(40,50,50);
            uScalar = new Scalar(80,255,255);
            colors.add(new ColorRange("Green", new Scalar(40,50,50), new Scalar(80, 255, 255)));
        }
        if(showOrange){
            lScalar = new Scalar(110,125,185);
            uScalar = new Scalar(150,255,255);
            colors.add(new ColorRange("Orange", new Scalar(110,125,185), new Scalar(150,255,255)));
        }

        if(showPurlple){
            lScalar = new Scalar(130,50,50);
            uScalar = new Scalar(170,255,255);
            colors.add(new ColorRange("Purple", new Scalar(130,50,50), new Scalar(170,255,255)));
        }
    }
    @Override
    public Mat processFrame(Mat input) {
        for(ColorRange cr : colors){
            if(this.isColor(input, cr.Lower, cr.Upper)) {
                this.ColorSeen = cr.Color;
                telemetry.addData("Color Seen: ", cr.Color);
                telemetry.update();
            }
        }

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
        //return mask;
        return input;
    }

    private boolean isColor(Mat input, Scalar lowerScalar, Scalar upperScalar){
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_BGR2HSV);

        Mat mask = new Mat();
        Core.inRange(hsvMat, lowerScalar, upperScalar, mask);

        //remove noise
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, new Mat());
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, new Mat());

        List<MatOfPoint> contourPoints = new ArrayList<>();
        Mat hierarchyMat = new Mat();
        Imgproc.findContours(mask, contourPoints, hierarchyMat, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int x,y;
        double area = 0.0;
        //List<Moments> moments = new ArrayList<Moments>(contourPoints.size());
        if(contourPoints.size() >1) {
            for (int i = 0; i < contourPoints.size(); i++) {
                try {
                    for (MatOfPoint contour : contourPoints) {
                        area = this.getArea(contour);
                    }
                } catch (Exception ex) {
                    telemetry.addData("EX ", ex.getMessage());
                }
            }
        }
        else if( contourPoints.size() == 1){
            area = this.getArea(contourPoints.get(0));
        }
        else
            area = 0.0;

        telemetry.addData("Area of: ", area);
        /*
         * Release the reusable Mat so that old data doesn't
         * affect the next step in the current processing
         */
        hsvMat.release();
        mask.release();
        hierarchyMat.release();

        return area > AREA_THRESHOLD;
    }

    private double getArea(MatOfPoint contour)
    {
        Point[] contourArrayGreen = contour.toArray();
        MatOfPoint2f areaPointsGreen = new MatOfPoint2f(contourArrayGreen);
        Rect rect = Imgproc.boundingRect(areaPointsGreen);
        //Imgproc.rectangle(mask, rect, new Scalar(50, 50, 50), 5);
        return rect.area();
    }
}