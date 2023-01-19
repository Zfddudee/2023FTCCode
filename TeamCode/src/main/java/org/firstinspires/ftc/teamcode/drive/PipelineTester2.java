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
    public OpenCvCamera webcam;
    public OpenCvCamera webcam2;
    @Override
    public void runOpMode() throws InterruptedException {
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FalconCam2"));
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FalconCam"));

        VisionSubsytem vision = new VisionSubsytem();
        vision.VisionSubsystem(webcam, webcam2);

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
