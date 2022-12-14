package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.ImageDetectorPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "CycleLeft", group="Bertha", preselectTeleOp = "BerthaTeleOp")
public class AutoCycleLeft extends LinearOpMode{

    OpenCvCamera webcam;
    ImageDetectorPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //region Variables

        //start bot at pose x = 30, y = 64, heading 90 degrees
        Pose2d startPose = new Pose2d(35, 61, Math.toRadians(90));

        // These are the headings
        double d1 = 90;
        double d2 = 0;

        // This is an x value
        double c1 = 35;
        // This is a y value
        double c2 = 11;
        // This is an x value
        double c3 = 45;
        // This is an x value
        double c4 = 58;
        // This is a y value
        double c5 = 22;
        // This is an x value
        double c6 = 12;

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


        //region TrajectoryOut
        TrajectorySequence TrajectoryOut = drive.trajectorySequenceBuilder(startPose)

                .lineToSplineHeading(new Pose2d(c1, c2, Math.toRadians(d1)))
                .turn(Math.toRadians(-d1))
                .lineToSplineHeading(new Pose2d(c3, c2, Math.toRadians(d2)))

                .build();

        //endregion

        //region TrajectoryX
        TrajectorySequence TrajectoryX = drive.trajectorySequenceBuilder(TrajectoryOut.end())

                .lineToLinearHeading(new Pose2d(c4, c2, Math.toRadians(d2)))
                .turn(Math.toRadians(d1))
                .lineToLinearHeading(new Pose2d(c4, c5, Math.toRadians(d1)))

                .build();

        //endregion

        //region TrajectoryY
        TrajectorySequence TrajectoryY = drive.trajectorySequenceBuilder(TrajectoryOut.end())

                .lineToLinearHeading(new Pose2d(c1, c2, Math.toRadians(d2)))
                .turn(Math.toRadians(d1))
                .lineToLinearHeading(new Pose2d(c1, c5, Math.toRadians(d1)))

                .build();

        //endregion

        //region TrajectoryZ
        TrajectorySequence TrajectoryZ = drive.trajectorySequenceBuilder(TrajectoryOut.end())

                .lineToLinearHeading(new Pose2d(c6, c2, Math.toRadians(d2)))
                .turn(Math.toRadians(d1))
                .lineToLinearHeading(new Pose2d(c6, c5, Math.toRadians(d1)))

                .build();

        //endregion

        Bertha bertha = new Bertha(hardwareMap, telemetry);

        waitForStart();

        //region ColorCode

        if (pipeline.ColorSeen == "Green") {
            bertha.AutoCheck();
            drive.followTrajectorySequence(TrajectoryOut);
            bertha.AutoExtake(Turret.TurretHorizontal.AutoLeft);
            bertha.AutoIntake(Constants.IntakeFlips1, Constants.IntakeOutAuto1);
            bertha.AutoExtake(Turret.TurretHorizontal.AutoLeft);
            bertha.AutoIntake(Constants.IntakeFlips2, Constants.IntakeOutAuto2);
            bertha.AutoExtake(Turret.TurretHorizontal.AutoLeft);
//            bertha.AutoIntake(Constants.IntakeFlips3, Constants.IntakeOutAuto3);
//            bertha.AutoExtakeRight();
//            bertha.AutoIntake(Constants.IntakeFlips4, Constants.IntakeOutAuto4);
//            bertha.AutoExtakeRight();
//            bertha.AutoIntake(Constants.IntakeFlips5, Constants.IntakeOutAuto5);
//            bertha.AutoExtakeRight();
            bertha.AutoReturn();
            drive.followTrajectorySequence(TrajectoryX);

        } else if (pipeline.ColorSeen == "Orange") {
            bertha.AutoCheck();
            drive.followTrajectorySequence(TrajectoryOut);
            bertha.AutoExtake(Turret.TurretHorizontal.AutoLeft);
            bertha.AutoIntake(Constants.IntakeFlips1, Constants.IntakeOutAuto1);
            bertha.AutoExtake(Turret.TurretHorizontal.AutoLeft);
            bertha.AutoIntake(Constants.IntakeFlips2, Constants.IntakeOutAuto2);
            bertha.AutoExtake(Turret.TurretHorizontal.AutoLeft);
//            bertha.AutoIntake(Constants.IntakeFlips3, Constants.IntakeOutAuto3);
//            bertha.AutoExtakeRight();
//            bertha.AutoIntake(Constants.IntakeFlips4, Constants.IntakeOutAuto4);
//            bertha.AutoExtakeRight();
//            bertha.AutoIntake(Constants.IntakeFlips5, Constants.IntakeOutAuto5);
//            bertha.AutoExtakeRight();
            bertha.AutoReturn();
            drive.followTrajectorySequence(TrajectoryY);


        } else if (pipeline.ColorSeen == "Purple") {
            bertha.AutoCheck();
            drive.followTrajectorySequence(TrajectoryOut);
            bertha.AutoExtake(Turret.TurretHorizontal.AutoLeft);
            bertha.AutoIntake(Constants.IntakeFlips1, Constants.IntakeOutAuto1);
            bertha.AutoExtake(Turret.TurretHorizontal.AutoLeft);
            bertha.AutoIntake(Constants.IntakeFlips2, Constants.IntakeOutAuto2);
            bertha.AutoExtake(Turret.TurretHorizontal.AutoLeft);
//            bertha.AutoIntake(Constants.IntakeFlips3, Constants.IntakeOutAuto3);
//            bertha.AutoExtakeRight();
//            bertha.AutoIntake(Constants.IntakeFlips4, Constants.IntakeOutAuto4);
//            bertha.AutoExtakeRight();
//            bertha.AutoIntake(Constants.IntakeFlips5, Constants.IntakeOutAuto5);
//            bertha.AutoExtakeRight();
            bertha.AutoReturn();
            drive.followTrajectorySequence(TrajectoryZ);
        }

        //endregion

        while (opModeIsActive()) {

//                telemetry.addData("placement]", pipeline.ColorSeen);
//                telemetry.update();
            //sleep(50);

        }
    }

}
