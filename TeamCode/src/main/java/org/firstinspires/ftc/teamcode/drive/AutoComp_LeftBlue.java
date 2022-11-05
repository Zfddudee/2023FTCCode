package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.VariableTunerTest;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class AutoComp_LeftBlue {

    @Autonomous(name = "AutoComp_LeftBlue", group = "RoadRunner/OpenCv", preselectTeleOp = "19589_TeleOp 2022-01-01")
    public class LeftBlue extends LinearOpMode {

        private DcMotor Lift;
        private DcMotor Intake;
        private Servo Bucket;
        private Servo TapeUpDown;
        private Servo TapeLeftRight;




        OpenCvCamera webcam;
        VariableTunerTest pipeline;
        @Override
        public void runOpMode() throws InterruptedException{
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Lift = hardwareMap.get(DcMotor.class, "LiftMotor");
            Intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
            Bucket = hardwareMap.get(Servo.class, "Bucket_Servo");
            TapeUpDown = hardwareMap.get(Servo.class, "tapeUpDown");
            TapeLeftRight = hardwareMap.get(Servo.class, "tapeLeftRight");



            //start bot at pose x = 30, y = 64, heading 90 degrees
            Pose2d startPose = new Pose2d(35, 62, Math.toRadians(270));

            //start bot at pose x = 12, y = 0, heading 90 degrees
            Pose2d endPose = new Pose2d(12, 0, Math.toRadians(270));

            // These are the headings
            double d1 = 0;
            double d2 = 180;
            double d3 = 90;
            double d4 = 270;

            // This is an x value
            double c1 = 35;
            // This is a y value
            double c2 = 30;
            // This is a y value
            double c3 = 16;
            // This is an x value
            double c4 = 40;
            // This is a y value
            double c5 = 12;
            // This is an x value
            double c6 = 47.125;
            // This is an x value
            double c7 = 46.375;
            // This is an x value
            double c8 = 45.875;
            // This is an x value
            double c9 = 45.775;
            // This is an x value
            double c10 = 50;
            // This is an x value
            double c11 = 58;
            // This is an x value
            double c12 = 60;
            // This is a y value
            double c13 = 14;
            // This is a y value
            double c14 = 36;
            // This is an x value
            double c15 = 37;
            // This is an x value
            double c16 = 14;
            // This is an x value
            double c17 = 12;
            
            
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


                                             public void onError(int errorCode) {

                                             }
                                         }

            );

            //drive.trajectoryBuilder(new Pose2d()).addTemporalMarker(3, () -> {Bucket.setPosition(intaking);}).build();
//.UNSTABLE_addTemporalMarkerOffset(0, () -> )
            //trajectory0
            TrajectorySequence Trajectory0 = drive.trajectorySequenceBuilder(startPose)

                    // This goes from the origin to the first extake position
                    .lineToSplineHeading(new Pose2d(c1, c2, Math.toRadians(d2)))
                    .lineToSplineHeading(new Pose2d(c1,c3, Math.toRadians(d2)))
                    .splineToConstantHeading(new Vector2d(c4,c5), Math.toRadians(d1))
                    .lineToSplineHeading(new Pose2d(c6, c5, Math.toRadians(d2)))

                    .waitSeconds(1)

                    // This is the movement for extake #2
                    .lineToSplineHeading(new Pose2d(c7, c5, Math.toRadians(d2)))

                    .waitSeconds(1)

                    // This is the movement for extake #3
                    .lineToSplineHeading(new Pose2d(c8, c5, Math.toRadians(d2)))

                    .waitSeconds(1)

                    // This is the movement for extake #4 and #5
                    // DO THE INTAKE AND EXTAKE MOTIONS TWICE FOR THIS STEP BECAUSE POSITION STAYS THE SAME FOR TWO ROUNDS
                    .lineToSplineHeading(new Pose2d(c9,c5, Math.toRadians(d2)))

                    .waitSeconds(2)
                    
                            .build();

            TrajectorySequence TrajectoryX = drive.trajectorySequenceBuilder(endPose)
                    .lineToSplineHeading(new Pose2d(c10, c5, Math.toRadians( d2)))
                            .lineToSplineHeading(new Pose2d(c11,c5, Math.toRadians(d4)))
                            .splineToConstantHeading(new Vector2d(c12, c13), Math.toRadians(d3))
                            .lineToSplineHeading(new Pose2d(c12, c14, Math.toRadians(d4)))
            
                            .build();
            
            TrajectorySequence TrajectoryY = drive.trajectorySequenceBuilder(endPose)
                    .lineToSplineHeading(new Pose2d(c15, c5, Math.toRadians(d4)))
                    .splineToConstantHeading(new Vector2d(c1, c13), Math.toRadians(d3))
                    .lineToSplineHeading(new Pose2d(c1, c14, Math.toRadians(d4)))
                    
                            .build();
            
            TrajectorySequence TrajectoryZ = drive.trajectorySequenceBuilder(endPose)
                    .lineToSplineHeading(new Pose2d(c16, c5, Math.toRadians(d4)))
                    .splineToConstantHeading(new Vector2d(c17, c13), Math.toRadians(d3))
                    .lineToSplineHeading(new Pose2d(c17, c14, Math.toRadians(d4)))

                            .build();

            waitForStart();
            if(pipeline.Last == 0) {
                drive.followTrajectorySequence(TrajectoryX);

            }
            else if(pipeline.Last == 1) {
                drive.followTrajectorySequence(TrajectoryY);

            }

            else if(pipeline.Last == 2) {
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
