package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.ImageDetectorPipeline;
//import org.firstinspires.ftc.teamcode.drive.opmode.Vision.VariableTunerTest;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
    @Disabled
    @Autonomous(name = "AutoComp_LeftBlue", group="Bertha",preselectTeleOp = "BerthaTeleOp")
    public class AutoComp_LeftBlue extends LinearOpMode {

        OpenCvCamera webcam;
        ImageDetectorPipeline pipeline;

        @Override
        public void runOpMode() throws InterruptedException {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            //start bot at pose x = 30, y = 64, heading 90 degrees
            Pose2d startPose = new Pose2d(35, 62, Math.toRadians(90));

            double d1 = 90;


            //region Park Left

            // This is an x value
            double c1 = 35;
            // This is a y value
            double c2 = 61;
            // This is an x value
            double c3 = 58;
            // This is an x value
            double c4 = 60;
            // This is a y value
            double c5 = 59;
            // This is a y value
            double c6 = 23;

            //endregion

            // The constant values for center parking ard duplicate values

            //region Park Right

            // This is an x value
            double c7 = 13;
            // This is an x value
            double c8 = 11;

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

            TrajectorySequence TrajectoryX = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(c3, c2, Math.toRadians(d1)))
                    .splineToConstantHeading(new Vector2d(c4, c5), Math.toRadians(d1))
                    .lineToSplineHeading(new Pose2d(c4, c6, Math.toRadians(d1)))

                    .build();

            TrajectorySequence TrajectoryY = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(c1, c6, Math.toRadians(d1)))

                    .build();

            TrajectorySequence TrajectoryZ = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(c7, c2, Math.toRadians(d1)))
                    .splineToConstantHeading(new Vector2d(c8, c5), Math.toRadians(d1))
                    .lineToSplineHeading(new Pose2d(c8, c6, Math.toRadians(d1)))

                    .build();

            waitForStart();

            if (pipeline.ColorSeen == "Green") {
                drive.followTrajectorySequence(TrajectoryX);
            }

            else if (pipeline.ColorSeen == "Orange") {
                drive.followTrajectorySequence(TrajectoryY);
            }
            
            else if (pipeline.ColorSeen == "Purple") {
                drive.followTrajectorySequence(TrajectoryZ);
            } else {
                {
                }
                while (opModeIsActive()) {
                    // telemetry.addData("placement]", Pipeline.Last);
                    // telemetry.update();
                    //sleep(50);


                }
            }
        }
    }


