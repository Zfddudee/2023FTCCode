package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.ImageDetectorPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class AutoComp_RightBlue {

    @Autonomous(name = "AutoComp_RightBlue", group = "RoadRunner/OpenCv", preselectTeleOp = "19589_TeleOp 2022-01-01")
    public class LeftBlue extends LinearOpMode {

        private DcMotor Lift;
        private DcMotor Intake;
        private Servo Bucket;
        private Servo TapeUpDown;
        private Servo TapeLeftRight;


        OpenCvCamera webcam;
        ImageDetectorPipeline pipeline;

        @Override
        public void runOpMode() throws InterruptedException {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Lift = hardwareMap.get(DcMotor.class, "LiftMotor");
            Intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
            Bucket = hardwareMap.get(Servo.class, "Bucket_Servo");
            TapeUpDown = hardwareMap.get(Servo.class, "tapeUpDown");
            TapeLeftRight = hardwareMap.get(Servo.class, "tapeLeftRight");


            //start bot at pose x = 30, y = 64, heading 90 degrees
            Pose2d startPose = new Pose2d(-35, 62, Math.toRadians(270));


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
                    .lineToSplineHeading(new Pose2d(-36, 61, Math.toRadians(270)))
                    .lineToSplineHeading(new Pose2d(-58, 61, Math.toRadians(270)))
                    .splineToConstantHeading(new Vector2d(-60, 59), Math.toRadians(270))
                    .lineToSplineHeading(new Pose2d(-60, 35, Math.toRadians(270)))

                    .build();

            TrajectorySequence TrajectoryY = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(-35, 35, Math.toRadians(270)))

                    .build();

            TrajectorySequence TrajectoryZ = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(-13, 61, Math.toRadians(270)))
                    .splineToConstantHeading(new Vector2d(-11, 59), Math.toRadians(270))
                    .lineToSplineHeading(new Pose2d(-11, 35, Math.toRadians(270)))

                    .build();

            if (pipeline.ColorSeen == "Green") {
                drive.followTrajectorySequence(TrajectoryX);

            } else if (pipeline.ColorSeen == "Orange") {
                drive.followTrajectorySequence(TrajectoryY);

            } else if (pipeline.ColorSeen == "Purple") {
                drive.followTrajectorySequence(TrajectoryZ);
            }
            while (opModeIsActive()) {
                // telemetry.addData("placement]", Pipeline.Last);
                // telemetry.update();
                //sleep(50);

            }
        }
    }
}


