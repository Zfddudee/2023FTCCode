package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.ImageDetectorPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "AutoOneCycle_RightBlue")
public class AutoOneCycle_RightBlue extends LinearOpMode{

    OpenCvCamera webcam;
    ImageDetectorPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //start bot at pose x = 30, y = 64, heading 90 degrees
        Pose2d startPose = new Pose2d(-35, -61, Math.toRadians(270));

        double d1 = 270;


        //region Park Left

        // This is an x value
        double c1 = -35;
        // This is a y value
        double c2 = 13;

        //endregion

        // The constant values for center parking ard duplicate values

        //region Park Right

        // This is an x value
        double c7 = -13;
        // This is an x value
        double c8 = -11;

        drive.setPoseEstimate(startPose);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FalconCam"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        pipeline = new ImageDetectorPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                                         }


                                         public void onError(int errorCode) {

                                         }
                                     }

        );


        //region TrajectoryX
        TrajectorySequence TrajectoryOut = drive.trajectorySequenceBuilder(startPose)

                .lineToSplineHeading(new Pose2d(c1, c2, Math.toRadians(d1)))

                .build();


        TrajectorySequence TrajectoryX = drive.trajectorySequenceBuilder(startPose)


                .build();

        //endregion

        //region TrajectoryY
        TrajectorySequence TrajectoryY = drive.trajectorySequenceBuilder(startPose)


                .build();

        //endregion

        //region TrajectoryZ
        TrajectorySequence TrajectoryZ = drive.trajectorySequenceBuilder(startPose)

                .build();

        //endregion

        Bertha bertha = new Bertha(hardwareMap, telemetry);

        waitForStart();

        if (pipeline.ColorSeen == "Green") {
            bertha.AutoCheck();
            drive.followTrajectorySequence(TrajectoryOut);
            bertha.AutoExtake();
            bertha.AutoIntake();
            bertha.AutoIntake1();
            bertha.AutoExtake();
            drive.followTrajectorySequence(TrajectoryX);

        } else if (pipeline.ColorSeen == "Orange") {
            drive.followTrajectorySequence(TrajectoryY);

        } else if (pipeline.ColorSeen == "Purple") {
            drive.followTrajectorySequence(TrajectoryZ);
        }
        while (opModeIsActive()) {

//                telemetry.addData("placement]", pipeline.ColorSeen);
//                telemetry.update();
            //sleep(50);

        }
    }
}