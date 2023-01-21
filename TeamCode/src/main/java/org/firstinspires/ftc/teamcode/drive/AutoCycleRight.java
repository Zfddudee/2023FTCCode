package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.ImageDetectorPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "CycleRight", group="Bertha", preselectTeleOp = "BerthaTeleOp")
public class AutoCycleRight extends LinearOpMode{

    enum State{
        PlacingCone,
        OpeningClaw,
        PreCone,
        None
    }
    OpenCvCamera webcam;
    ImageDetectorPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //region Variables

        Pose2d startPose = new Pose2d(-35, -61, Math.toRadians(270));

        double d1 = 270;
        double d2 = 0;

        //region

        // This is an x value
        double c1 = -35;
        // This is a y value
        double c2 = -30;
        // This is an x value
        double c3 = -45;
        // This is a y value
        double c4 = -11.5;
        // This is an x value
        double c5 = -52;

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
                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                                         }


                                         public void onError(int errorCode) {

                                         }
                                     }
        );

//        //region TrajectoryOut
//        TrajectorySequence TrajectoryOut = drive.trajectorySequenceBuilder(startPose)
//
//                .lineToSplineHeading(new Pose2d(c1, c2, Math.toRadians(d1)))
//                .splineTo(new Vector2d(c3, c4), Math.toRadians(d2))
//                .lineToSplineHeading(new Pose2d(c5, c4, Math.toRadians(d2)))
//                .build();
//        //endregion

//        //region TrajectoryX
//        TrajectorySequence TrajectoryX = drive.trajectorySequenceBuilder(TrajectoryOut.end())
//                .lineToLinearHeading(new Pose2d(-59, c4, Math.toRadians(d3)))
//                .turn(Math.toRadians(-90))
//                .lineToLinearHeading(new Pose2d(-59, -22, Math.toRadians(d1)))
//                .build();
//
//        //endregion
//
//        //region TrajectoryY
//        TrajectorySequence TrajectoryY = drive.trajectorySequenceBuilder(TrajectoryOut.end())
//
//                .lineToLinearHeading(new Pose2d(-35, -11, Math.toRadians(d3)))
//                .turn(Math.toRadians(-90))
//                .lineToLinearHeading(new Pose2d(-35, -22, Math.toRadians(d1)))
//
//                .build();
//
//        //endregion
//
//        //region TrajectoryZ
//        TrajectorySequence TrajectoryZ = drive.trajectorySequenceBuilder(TrajectoryOut.end())
//
//                .lineToLinearHeading(new Pose2d(-11.5, c4, Math.toRadians(d3)))
//                .turn(Math.toRadians(-90))
//                .lineToLinearHeading(new Pose2d(-11.5, -22, Math.toRadians(d1)))
//
//                .build();
//
//        //endregion

        BerthaAuto bertha = new BerthaAuto(hardwareMap, telemetry);

        waitForStart();

        String pipelineColorSeen = pipeline.ColorSeen;
        bertha.AutoCheck();
//        drive.followTrajectorySequence(TrajectoryOut);

//
//
//        if (pipeline.ColorSeen == "Green") {
//            bertha.AutoCheck();
//            drive.followTrajectorySequence(TrajectoryOut);
//            bertha.AutoExtake(Turret.TurretHorizontal.AutoRight);
//            bertha.AutoIntake(Constants.IntakeFlips1, Constants.IntakeOutAuto1, Turret.TurretHorizontal.AutoRight);
//            bertha.AutoIntake(Constants.IntakeFlips2, Constants.IntakeOutAuto2, Turret.TurretHorizontal.AutoRight);
////            bertha.AutoIntake(Constants.IntakeFlips3, Constants.IntakeOutAuto3);
////            bertha.AutoExtakeLeft();
////            bertha.AutoIntake(Constants.IntakeFlips4, Constants.IntakeOutAuto4);
////            bertha.AutoExtakeLeft();
////            bertha.AutoIntake(Constants.IntakeFlips5, Constants.IntakeOutAuto5);
////            bertha.AutoExtakeLeft();
////            bertha.AutoIntake();
////            bertha.AutoExtake();
//            bertha.AutoReturn();
//            drive.followTrajectorySequence(TrajectoryX);
//
//        } else if (pipeline.ColorSeen == "Orange") {
//           bertha.AutoCheck();
//            drive.followTrajectorySequence(TrajectoryOut);
//            bertha.AutoExtake(Turret.TurretHorizontal.AutoRight);
//            bertha.AutoIntake(Constants.IntakeFlips1, Constants.IntakeOutAuto1, Turret.TurretHorizontal.AutoRight);
//            bertha.AutoIntake(Constants.IntakeFlips2, Constants.IntakeOutAuto2, Turret.TurretHorizontal.AutoRight);
////            bertha.AutoIntake(Constants.IntakeFlips3, Constants.IntakeOutAuto3);
////            bertha.AutoExtakeLeft();
////            bertha.AutoIntake(Constants.IntakeFlips4, Constants.IntakeOutAuto4);
////            bertha.AutoExtakeLeft();
////            bertha.AutoIntake(Constants.IntakeFlips5, Constants.IntakeOutAuto5);
////            bertha.AutoExtakeLeft();
////            bertha.AutoIntake();
////            bertha.AutoExtake();
//            bertha.AutoReturn();
//            drive.followTrajectorySequence(TrajectoryY);
//
//
//        } else if (pipeline.ColorSeen == "Purple") {
//            bertha.AutoCheck();
//            drive.followTrajectorySequence(TrajectoryOut);
//            bertha.AutoExtake(Turret.TurretHorizontal.AutoRight);
//            bertha.AutoIntake(Constants.IntakeFlips1, Constants.IntakeOutAuto1, Turret.TurretHorizontal.AutoRight);
//            bertha.AutoIntake(Constants.IntakeFlips2, Constants.IntakeOutAuto2, Turret.TurretHorizontal.AutoRight);
////            bertha.AutoIntake(Constants.IntakeFlips3, Constants.IntakeOutAuto3);
////            bertha.AutoExtakeLeft();
////            bertha.AutoIntake(Constants.IntakeFlips4, Constants.IntakeOutAuto4);
////            bertha.AutoExtakeLeft();
////            bertha.AutoIntake(Constants.IntakeFlips5, Constants.IntakeOutAuto5);
////            bertha.AutoExtakeLeft();
////            bertha.AutoIntake();
////            bertha.AutoExtake();
//            bertha.AutoReturn();
//            drive.followTrajectorySequence(TrajectoryZ);
//
//        }
        ElapsedTime timer = new ElapsedTime();
        int coneCount = 0;

        //drop first cone
        bertha.lift.MoveLift(Lift.LiftHeight.High);
        bertha.turret.MoveVertical(Turret.TurretHeight.CycleVertical);
        bertha.turret.Wait(200);
        bertha.turret.MoveHorizontal(Turret.TurretHorizontal.AutoRight);
        bertha.turret.Wait(50);
        bertha.turret.MoveVertical(Turret.TurretHeight.Flipped);
        bertha.OpenClaw();
        coneCount++;
        bertha.turret.Wait(50);
        bertha.turret.MoveHorizontal(Turret.TurretHorizontal.Center);
        bertha.turret.Wait(100);
        bertha.turret.MoveVertical(Turret.TurretHeight.Default);
        bertha.lift.MoveLift(Lift.LiftHeight.Default);

        bertha.intakeHeightOffset = Constants.IntakeFlips1;
        bertha.PreConePickup();
        State currentState = State.PreCone;
        while (opModeIsActive() && coneCount <= Constants.ConeCount ) {
            if(bertha.intaking == Bertha.Intaking.IntakeFlip)
                bertha.intaking = Bertha.Intaking.IntakeFlipAuto;
            bertha.RunOpMode();
            if(bertha.intaking == Bertha.Intaking.None && currentState == State.PreCone){
                currentState = State.PlacingCone;
                timer.reset();
                timer.startTime();
                bertha.PlaceConeOverJunction();
            }
            if(timer.milliseconds() > 200 && currentState == State.PlacingCone) {
                bertha.OpenClaw();
                coneCount++;
                currentState = State.OpeningClaw;
            }
            else if(currentState == State.OpeningClaw && timer.milliseconds() > 500) {
                bertha.PreConePickup();
                currentState = State.PreCone;
                switch (coneCount){
                    case 2:
                        bertha.intakeHeightOffset = Constants.IntakeFlips2;
                        break;
                    case 3:
                        bertha.intakeHeightOffset = Constants.IntakeFlips3;
                        break;
                    case 4:
                        bertha.intakeHeightOffset = Constants.IntakeFlips4;
                        break;
                    case 5:
                        bertha.intakeHeightOffset = Constants.IntakeFlips5;
                }
            }
        }

        bertha.Reset();
        //Park

    }
}