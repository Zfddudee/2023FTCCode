package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.VariableTunerTest;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RedWarehouseRoadRunner", group = "RoadRunner/OpenCv", preselectTeleOp = "19589_TeleOp 2022-01-01")
public class RROCVTracking extends LinearOpMode {


    OpenCvCamera webcam;
    VariableTunerTest pipeline;
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //start bot at pose x = -30, y = -64, heading 90 degrees
        Pose2d startPose = new Pose2d(10, -64, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FalconCam"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        pipeline = new VariableTunerTest(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                                         }

                                         @Override
                                         public void onError(int errorCode) {

                                         }
                                     }

        );

        //TrajectoryTest2
        TrajectorySequence TrajectoryTest1 = drive.trajectorySequenceBuilder(startPose)
                .turn(5)
                .build();
        TrajectorySequence TrajectoryTest2 = drive.trajectorySequenceBuilder(startPose)
                .turn(-5)
                .build();
        TrajectorySequence TrajectoryTest3 = drive.trajectorySequenceBuilder(startPose)
                .forward(2)
                .build();
        TrajectorySequence TrajectoryTest4 = drive.trajectorySequenceBuilder(startPose)
                .back(2)
                .build();

        waitForStart();
        if(pipeline.xOrange <= 135) {
            drive.followTrajectorySequence(TrajectoryTest1);
        }
        else if(pipeline.xOrange >= 165) {
            drive.followTrajectorySequence(TrajectoryTest2);
        }

        else if(pipeline.areaOrange <= 700) {
            drive.followTrajectorySequence(TrajectoryTest3);
        }
        else if(pipeline.areaOrange >= 750) {
            drive.followTrajectorySequence(TrajectoryTest4);
        }
        while (opModeIsActive()) {
            // telemetry.addData("placement]", Pipeline.Last);
            // telemetry.update();
            //sleep(50);
        }
    }


}