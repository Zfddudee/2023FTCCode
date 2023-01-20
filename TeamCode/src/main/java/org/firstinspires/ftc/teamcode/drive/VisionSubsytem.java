package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.ImageDetectorPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.JunctionPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.JunctionPipeline2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class VisionSubsytem{
    public static class VisionConstants {
        public static int WIDTH = 320;
        public static int HEIGHT = 240;
        public static OpenCvCameraRotation ROTATION = OpenCvCameraRotation.UPSIDE_DOWN;
    }

    public double X1(){return juntionPipeline.x;}
    public double X2(){return juntionPipeline2.x;}
    public double XE1(){return juntionPipeline.xError;}
    public double XE2(){return juntionPipeline2.xError;}
    public String ColorSeen(){return readerPipeline.ColorSeen;}


    Telemetry telemetry;
    public JunctionPipeline juntionPipeline = new JunctionPipeline();
    public JunctionPipeline2 juntionPipeline2 = new JunctionPipeline2();
    public ImageDetectorPipeline readerPipeline = new ImageDetectorPipeline(telemetry);

    public OpenCvCamera webcam;
    public OpenCvCamera webcam2;


    public void VisionSubsystem(OpenCvCamera c, OpenCvCamera d) {
        webcam = c;
        webcam2 = d;
    }
    public void startStreaming1(){
        webcam.startStreaming(VisionConstants.WIDTH, VisionConstants.HEIGHT, VisionConstants.ROTATION);
    }
    public void startStreaming2(){
        webcam2.startStreaming(VisionConstants.WIDTH, VisionConstants.HEIGHT, VisionConstants.ROTATION);
    }
    public void startReaderPipeline() {
        webcam.setPipeline(readerPipeline);
        webcam2.setPipeline(null);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() { startStreaming1(); Constants.CameraOpened1 = true; } public void onError(int errorCode) {Constants.CameraOpened1 = false;}});
        webcam2.closeCameraDeviceAsync(()->{Constants.CameraOpened2 = false;});
        Constants.CameraState = "Reader";
    }
    public void startJunctionPipeline() {
//        webcam.setPipeline(juntionPipeline);
//        webcam2.setPipeline(juntionPipeline2);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override public void onOpened() { startStreaming1();  Constants.CameraOpened1 = true;} public void onError(int errorCode) {Constants.CameraOpened1 = false;}});
//        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override public void onOpened() { startStreaming2();  Constants.CameraOpened2 = true;} public void onError(int errorCode) {Constants.CameraOpened2 = true;}});
//        Constants.CameraState = "Junction";
    }

    public void stopPipeline() {
//        webcam.setPipeline(null);
//        webcam2.setPipeline(null);
//        webcam.closeCameraDeviceAsync(()->{});
//        webcam2.closeCameraDeviceAsync(()->{});
//        Constants.CameraOpened1 = false;
//        Constants.CameraOpened2 = false;
//        Constants.CameraState = "None";
    }
}
