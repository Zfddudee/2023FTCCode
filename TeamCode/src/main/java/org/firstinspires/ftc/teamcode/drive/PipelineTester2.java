package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.JunctionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "PipelineTester2", group="Test")

public class PipelineTester2 extends LinearOpMode {
    public OpenCvCamera webcam; //Need to add
    public OpenCvCamera webcam2; //Need to add
    @Override
    public void runOpMode() throws InterruptedException {
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FalconCam")); //Need to add
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FalconCam2")); //Need to add

        VisionSubsytem vision = new VisionSubsytem(); //Need to add
        vision.VisionSubsystem(webcam, webcam2); //Need to add
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {         webcam.startStreaming(VisionSubsytem.VisionConstants.WIDTH, VisionSubsytem.VisionConstants.HEIGHT, VisionSubsytem.VisionConstants.ROTATION); Constants.CameraOpened1 = true;} public void onError(int errorCode) {Constants.CameraOpened1 = false;}});
        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {         webcam2.startStreaming(VisionSubsytem.VisionConstants.WIDTH, VisionSubsytem.VisionConstants.HEIGHT, VisionSubsytem.VisionConstants.ROTATION);Constants.CameraOpened2 = true;} public void onError(int errorCode) {Constants.CameraOpened2 = false;}});

        while(!isStopRequested()){
            if(gamepad1.a){
                vision.startReaderPipeline();
            }
            if(gamepad1.b){
                vision.startJunctionPipeline();
            }
            if(gamepad1.y){
                vision.stopPipeline();
            }
            telemetry.addData("[>]", "Change these values in tuner menu");
//            telemetry.addData("[Lower Scalar]", pipeline.lower);
//            telemetry.addData("[Upper Scalar]", pipeline.upper);
//            telemetry.addData("[x Error]", pipeline.xError);
//            telemetry.addData("[Top X]", pipeline.X);
//            telemetry.addData("[Top Y]", pipeline.Y);
//            telemetry.addData("[Centroid X]", pipeline.x);
//            telemetry.addData("[Centroid Y]", pipeline.y);
            telemetry.addData("Color Seen = ", vision.ColorSeen());
            telemetry.addData("Camera State = ", Constants.CameraState);
            telemetry.addData("camera1 Opened?", Constants.CameraOpened1);
            telemetry.addData("camera2 Opened?", Constants.CameraOpened2);
            telemetry.addData("X1", vision.X1());
            telemetry.addData("X2", vision.X2());
            telemetry.addData("XE1", vision.XE1());
            telemetry.addData("XE2", vision.XE2());
            telemetry.update();
        }
    }
}
