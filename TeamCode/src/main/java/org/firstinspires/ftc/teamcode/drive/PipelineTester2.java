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

    OpenCvCamera webcam;
    JunctionPipeline pipeline;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FalconCam2"), cameraMonitorViewId);
        pipeline = new JunctionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                                         }


                                         public void onError(int errorCode) {

                                         }
                                     }

        );

        while(!isStopRequested()){

            telemetry.addData("[>]", "Change these values in tuner menu");
            telemetry.addData("[Lower Scalar]", pipeline.lower);
            telemetry.addData("[Upper Scalar]", pipeline.upper);
            telemetry.addData("[x Error]", pipeline.xError);
            telemetry.addData("[Top X]", pipeline.X);
            telemetry.addData("[Top Y]", pipeline.Y);
            telemetry.addData("[Centroid X]", pipeline.x);
            telemetry.addData("[Centroid Y]", pipeline.y);
            telemetry.update();
        }
    }
}
