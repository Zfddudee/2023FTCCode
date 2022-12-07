package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.ImageDetectorPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "AutoOneCycle_RightBlue", group="Bertha")
public class AutoOneCycle_RightBlue extends LinearOpMode{

    OpenCvCamera webcam;
    ImageDetectorPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //region Variables

        //start bot at pose x = 30, y = 64, heading 90 degrees
        Pose2d startPose = new Pose2d(-35, -61, Math.toRadians(270));

        double d1 = 270;
        double d2 = 180;
        double d3 = 0;

        //region Park Left

        // This is an x value
        double c1 = -35;
        // This is a y value
        double c2 = -11;
        // This is an x value
        double c3 = -37;
        // This is a y value
        double c4 = -11;
        // This is an x value
        double c5 = -25;

        //endregion

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
                .splineToConstantHeading(new Vector2d(c3, c4), Math.toRadians(d2))
                .lineToSplineHeading(new Pose2d(c5, c4, Math.toRadians(d3)))

                .build();

        TrajectorySequence TrajectoryX = drive.trajectorySequenceBuilder(TrajectoryOut.end())

                .lineToLinearHeading(new Pose2d(-59, c4, Math.toRadians(d3)))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(-59, -22, Math.toRadians(d1)))

                .build();

        //endregion

        //region TrajectoryY
        TrajectorySequence TrajectoryY = drive.trajectorySequenceBuilder(TrajectoryOut.end())

                .lineToLinearHeading(new Pose2d(-35, -11, Math.toRadians(d3)))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(-35, -22, Math.toRadians(d1)))

                .build();

        //endregion

        //region TrajectoryZ
        TrajectorySequence TrajectoryZ = drive.trajectorySequenceBuilder(TrajectoryOut.end())

                .lineToLinearHeading(new Pose2d(-11.5, c4, Math.toRadians(d3)))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(-11.5, -22, Math.toRadians(d1)))

                .build();

        //endregion

        Bertha bertha = new Bertha(hardwareMap, telemetry);

        waitForStart();

        if (pipeline.ColorSeen == "Green") {
            bertha.AutoCheck();
            drive.followTrajectorySequence(TrajectoryOut);
            bertha.AutoExtakeLeft();
            bertha.AutoIntake();
            bertha.AutoExtakeLeft();
//            bertha.AutoIntake();
//            bertha.AutoExtake();
            bertha.AutoReturn();
            drive.followTrajectorySequence(TrajectoryX);

        } else if (pipeline.ColorSeen == "Orange") {
           bertha.AutoCheck();
            drive.followTrajectorySequence(TrajectoryOut);
            bertha.AutoExtakeLeft();
            bertha.AutoIntake();
            bertha.AutoExtakeLeft();
//            bertha.AutoIntake();
//            bertha.AutoExtake();
            bertha.AutoReturn();
            drive.followTrajectorySequence(TrajectoryY);


        } else if (pipeline.ColorSeen == "Purple") {
            bertha.AutoCheck();
            drive.followTrajectorySequence(TrajectoryOut);
            bertha.AutoExtakeLeft();
            bertha.AutoIntake();
            bertha.AutoExtakeLeft();
//            bertha.AutoIntake();
//            bertha.AutoExtake();
            bertha.AutoReturn();
            drive.followTrajectorySequence(TrajectoryZ);

        }
        while (opModeIsActive()) {

//                telemetry.addData("placement]", pipeline.ColorSeen);
//                telemetry.update();
            //sleep(50);

        }
    }
}