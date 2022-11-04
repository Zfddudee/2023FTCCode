package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.VariableTunerTest;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "LeftBlue", group = "RoadRunner/OpenCv", preselectTeleOp = "19589_TeleOp 2022-01-01")
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
        Pose2d startPose = new Pose2d(35, 60, Math.toRadians(270));

        //start bot at pose x = 12, y = 0, heading 90 degrees
        Pose2d endPose = new Pose2d(12, 0, Math.toRadians(270));

        int d1 = 270;

        int d2 = 0;

        int d3 = 180;

        int d4 = 180;

        // These values should be x = Positive, y = Positive

        // This is an x value
        int c1 = 12;
        // This is a y-value
        int c2 = 2;
        // This is an x-value
        int c3 = 16;
        // This is a y-value
        int c4 = 12;
        // This is an x-value
        int c5 = 25;
        // This is a y-value
        int c6 = 4;
        // This is a y-value
        int c7 = 0;
        // This is an x-value
        int c8 = 35;
        // This is a y-value
        int c9 = 20;
        // This is an x-value
        int c10 = 56;
        // This is an x-value
        int c11 = 54;
        // This is an x-value
        int c12 = 50;
        // This is an x-value
        int c13 = 46;
        // This is an x-value
        int c14 = 42;
        // This is a y-value
        int c15 = 36;
        // This is an x-value
        int c16 = 36;
        // This is a y-value
        int c17 = 32;
        // This is an x-value
        int c18 = 60;

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
                // The segment goes from the origin to the first extake
                .lineToSplineHeading(new Pose2d(c8, c9, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c5, c4), Math.toRadians(d3))
                .lineToSplineHeading(new Pose2d(c3, c4, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c1, c6), Math.toRadians(d1))
                .lineToSplineHeading(new Pose2d(c1, c7, Math.toRadians(d1)))
                .waitSeconds(1)

                // This segment goes from extake #1 to intake #1
                .lineToSplineHeading(new Pose2d(c1, c2, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c3, c4), Math.toRadians(d2))
                .lineToSplineHeading(new Pose2d(c10, c4, Math.toRadians(d4)))
                .waitSeconds(1)

                // This segment goes from intake #1 to extake #2
                .lineToSplineHeading(new Pose2d(c5, c4, Math.toRadians(d1)))
                .lineToSplineHeading(new Pose2d(c3, c4, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c1, c6), Math.toRadians(d1))
                .lineToSplineHeading(new Pose2d(c1, c7, Math.toRadians(d1)))
                .waitSeconds(1)

                // This segment goes from extake #2 to intake #2
                .lineToSplineHeading(new Pose2d(c1, c2, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c3, c4), Math.toRadians(d2))
                .lineToSplineHeading(new Pose2d(c11, c4, Math.toRadians(d4)))
                .waitSeconds(1)

                // This segment goes from intake #2 to extake #3
                .lineToSplineHeading(new Pose2d(c5, c4, Math.toRadians(d1)))
                .lineToSplineHeading(new Pose2d(c3, c4, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c1, c6), Math.toRadians(d1))
                .lineToSplineHeading(new Pose2d(c1, c7, Math.toRadians(d1)))
                .waitSeconds(1)

                // This segment goes from extake #3 to intake #3
                .lineToSplineHeading(new Pose2d(c1, c2, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c3, c4), Math.toRadians(d2))
                .lineToSplineHeading(new Pose2d(c12, c4, Math.toRadians(d4)))
                .waitSeconds(1)

                // This segment goes from intake #3 to extake #4
                .lineToSplineHeading(new Pose2d(c5, c4, Math.toRadians(d1)))
                .lineToSplineHeading(new Pose2d(c3, c4, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c1, c6), Math.toRadians(d1))
                .lineToSplineHeading(new Pose2d(c1, c7, Math.toRadians(d1)))
                .waitSeconds(1)

                // This segment goes from extake #4 to intake #4
                .lineToSplineHeading(new Pose2d(c1, c2, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c3, c4), Math.toRadians(d2))
                .lineToSplineHeading(new Pose2d(c13, c4, Math.toRadians(d4)))
                .waitSeconds(1)

                // This segment goes from intake #4 to extake #5
                .lineToSplineHeading(new Pose2d(c5, c4, Math.toRadians(d1)))
                .lineToSplineHeading(new Pose2d(c3, c4, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c1, c6), Math.toRadians(d1))
                .lineToSplineHeading(new Pose2d(c1, c7, Math.toRadians(d1)))
                .waitSeconds(1)

                // This segment goes from extake #5 to intake #5
                .lineToSplineHeading(new Pose2d(c1, c2, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c3, c4), Math.toRadians(d2))
                .lineToSplineHeading(new Pose2d(c14, c4, Math.toRadians(d4)))
                .waitSeconds(1)

                // This segment goes from intake #5 to extake #6
                .lineToSplineHeading(new Pose2d(c5, c4, Math.toRadians(d1)))
                .lineToSplineHeading(new Pose2d(c3, c4, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c1, c6), Math.toRadians(d1))
                .lineToSplineHeading(new Pose2d(c1, c7, Math.toRadians(d1)))
                .waitSeconds(1)

                .build();

        TrajectorySequence TrajectoryX = drive.trajectorySequenceBuilder(endPose)
                // This segment parks the robot in spot #1
                .lineToSplineHeading(new Pose2d(c1, c15, Math.toRadians(d1)))
                .build();

        TrajectorySequence TrajectoryY = drive.trajectorySequenceBuilder(endPose)
                // This segment parks the robot in spot #2
                .lineToSplineHeading(new Pose2d(c1, c17, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c3, c15), Math.toRadians(d2))
                .lineToSplineHeading(new Pose2d(c16, c15, Math.toRadians(d1)))
                .build();

        TrajectorySequence TrajectoryZ = drive.trajectorySequenceBuilder(endPose)
                // This segment parks the robot in spot #3
                .lineToSplineHeading(new Pose2d(c1, c17, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c3, c15), Math.toRadians(d2))
                .lineToSplineHeading(new Pose2d(c18, c15, Math.toRadians(d1)))


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
