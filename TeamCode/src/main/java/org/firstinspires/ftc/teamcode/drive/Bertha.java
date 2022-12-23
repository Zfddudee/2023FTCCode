package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.JunctionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
public class Bertha{

    enum State {
        None,
        PickAndExchange,
        AutoPickAndExchangeRight,
        AutoPickAndExchangeLeft,
        CameraCentering,
        ExchangeToExtake,
        IntakeReturn,
    };

    ///region Robot objects
    private Lift lift;
    private DriveTrain driveTrain;
    private Turret turret;
    private Intake intake;
    //endregion

    private ElapsedTime timer;
    private State state;
    private Telemetry telemetry;
    private IntakeScheduler intakeScheduler = new IntakeScheduler();
    private ExtakeScheduler extakeScheduler = new ExtakeScheduler();

    public Bertha(HardwareMap map, Telemetry tel){
        lift = new Lift(map, tel);
        driveTrain = new DriveTrain(map, tel);
        turret = new Turret(map, tel);
        intake = new Intake(map, tel);

        timer = new ElapsedTime();
        state = State.None;
        telemetry = tel;
    }

    private void LogAllTelemetry(){
        telemetry.addData("Current State: ", state);
        turret.Telemetry();
        intake.Telemetry();
        lift.Telemetry();
    }

    HardwareMap hardwareMap;
    OpenCvCamera webcam;
    JunctionPipeline pipeline;
    //region TeleOp
    public void RunOpMode() {
        //Todo may want to move this to bertha teleop if this does not work.

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FalconCam"), cameraMonitorViewId);


        pipeline = new JunctionPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
                                         }


                                         public void onError(int errorCode) {

                                         }
                                     }

        );
        intakeScheduler.start();
        extakeScheduler.start();
        switch (state)
        {
            case PickAndExchange:
                intake.AutoCloseClaw();
                if(intake.AutoCloseClaw()){
                    MoveToExchange2();
                    state = State.None;
                }
                break;
            case AutoPickAndExchangeRight:
                intake.AutoCloseClaw();
                if(intake.AutoCloseClaw() || timer.milliseconds() >= 750){
                    AutoMoveToExchange2(Turret.TurretHorizontal.AutoRight);
                    state = State.None;
                }
                break;
            case AutoPickAndExchangeLeft:
                intake.AutoCloseClaw();
                if(intake.AutoCloseClaw() || timer.milliseconds() >= 750){
                    AutoMoveToExchange2(Turret.TurretHorizontal.AutoLeft);
                    state = State.None;
                }
                break;
            case CameraCentering:
//                0.006 points per degree
                double turretPose;
                turretPose = pipeline.xErrorServo + Constants.TurretDefault;
                turret.MoveHorizontal(turretPose);
                break;

            default:
                break;
        }
        this.LogAllTelemetry();
    }



    //This moves the intake into a position to grab a cone in its low position
    public void PreConePickUp() {
        //        scheduler.schedule(() -> turret.SlideOut(), 0);
        intakeScheduler.stop();
        intakeScheduler.start();
        if(!turret.IsSlideOut()) {
            turret.SlideOut();
            intakeScheduler.schedule(() -> {
                intake.FlipDown();
                turret.MoveVertical(Turret.TurretHeight.Low);
            }, TimingConstants.Time1);
        } else {
                intake.FlipDown();
                turret.MoveVertical(Turret.TurretHeight.Low);
        }
        intakeScheduler.schedule(() -> intake.IntakeOut(), TimingConstants.Time2);
        intakeScheduler.schedule(() -> intake.OpenClaw(), TimingConstants.Time3);
        intakeScheduler.schedule(() -> {
            intake.OpenClaw();
            intake.SlideMotorOut();
            PickAndExchange();
//            intakeScheduler.stop();
        }, TimingConstants.Time4);

    }

    public void MoveToExchange2() {
//        intakeScheduler.stop();
//        intakeScheduler.start();
        lift.MoveLift(Lift.LiftHeight.Default);
        intake.CloseClaw();
        turret.SlideOut();
        turret.CloseClaw();
        turret.MoveVertical(Turret.TurretHeight.Default);
        intakeScheduler.schedule(() -> {
            turret.OpenClaw();
            intake.FlipUp();
        }, 150);
        intakeScheduler.schedule(() -> intake.SlideMotorExchange(), 200);
        intakeScheduler.schedule(() -> intake.IntakeNewExchange(), 200);
        intakeScheduler.schedule(() -> turret.SlideMid(), 450);
        intakeScheduler.schedule(() -> turret.CloseClaw(), 500);
        intakeScheduler.schedule(() -> intake.OpenClaw(), 50);
        intakeScheduler.schedule(() -> {
            lift.MoveLift(Lift.LiftHeight.Medium);
            intake.IntakeIn();
            turret.SlideIn();
            TeleOpCycle();
            intakeScheduler.stop();
        }, 100);
    }

    public void PickUpOverRide() {
        turret.SlideOut();
        intakeScheduler.schedule(() -> {
            turret.MoveVertical(Turret.TurretHeight.Low);
            intake.FlipDown();
    }, 500);
        intakeScheduler.schedule(() -> intake.IntakeOut(), 300);
        intakeScheduler.schedule(() -> intake.OpenClaw(), 500);
        intakeScheduler.schedule(() -> {
            intake.OpenClaw();
            intake.SlideMotorOut();
        }, 100);
    }

    public void ExchangeToExtake() {
        if(state != State.ExchangeToExtake) {
            state = State.ExchangeToExtake;
            lift.MoveLift(Lift.LiftHeight.High);
            turret.SlideOut();
            turret.MoveVertical(Turret.TurretHeight.Flipped);
            state = State.None;
        }
    }

    public boolean IntakeReturn() {
        extakeScheduler.start();
        if(state != State.IntakeReturn) {
            if (!turret.IsAtHorizontalPosition(Constants.TurretDefault, 5.0)) {
                turret.MoveHorizontal(Turret.TurretHorizontal.Center);
                extakeScheduler.schedule(() -> turret.MoveHorizontal(Turret.TurretHorizontal.Center), 750);
            }
            if(!turret.IsSlideOut()) {
                turret.SlideOut();
            }
            if(!turret.IsAtVerticalPosition(Constants.ExtakeFlipIn, 2.0)) {
                extakeScheduler.schedule(() -> turret.MoveVertical(Turret.TurretHeight.Default), 0);
            }
            turret.MoveHorizontal(Turret.TurretHorizontal.Center);
            turret.CloseClaw();
            extakeScheduler.schedule(() -> {
                turret.MoveVertical(Turret.TurretHeight.Default);
                lift.MoveLift(Lift.LiftHeight.Default);
                state = State.None;
            }, 250);
        }
        return true;
    }

    public void Reset() {
        turret.CloseClaw();
        intake.CloseClaw();
        lift.MoveLift(Lift.LiftHeight.Medium);
        intake.FlipUp();
        turret.SlideOut();
        intakeScheduler.schedule(() -> intake.IntakeIn(), 300);
        intakeScheduler.schedule(() -> {
            intake.SlideMotorIn();
            turret.MoveHorizontal(Turret.TurretHorizontal.Center);
        }, 300);
        intakeScheduler.schedule(() -> turret.SlideIn(), 500);
        intakeScheduler.schedule(() -> turret.MoveVertical(Turret.TurretHeight.Default), 500);
        intakeScheduler.schedule(() -> {
            turret.SlideIn();
            lift.MoveLift(Lift.LiftHeight.Default);
            turret.OpenClaw();
        }, 250);
    }

    public void TeleOpCycle() {
        extakeScheduler.start();
        lift.MoveLift(Lift.LiftHeight.High);
        extakeScheduler.schedule(() -> turret.MoveVertical(Turret.TurretHeight.CycleVertical), 300);
        extakeScheduler.schedule(() -> state = State.CameraCentering, 750);
//        extakeScheduler.schedule(() -> turret.MoveHorizontal(Turret.TurretHorizontal.CycleHorizontal), 750);
        extakeScheduler.schedule(() -> turret.MoveVertical(Turret.TurretHeight.Flipped), Constants.CycleDropDelay);
        extakeScheduler.schedule(() -> extakeScheduler.stop(), 0);
    }

    //endregion

    //region Autonomous
    public void AutoCheck() {
        turret.CloseClaw();
        turret.SlideIn();
    }

    public void AutoIntake(int ConePosition, int SlidePose, Turret.TurretHorizontal TurretSide) {
        intakeScheduler.start();
        turret.SlideOut();
        intakeScheduler.schedule(() -> {
            turret.MoveVertical(Turret.TurretHeight.Low);
            intake.FlipDown();
        }, 500);
        intakeScheduler.schedule(() -> intake.AutoIntakeOut(ConePosition), 300);
        intakeScheduler.schedule(() -> intake.OpenClaw(), 200);
        intakeScheduler.schedule(() -> intake.OpenClaw(), 100);
        intakeScheduler.schedule(() -> {
            intake.OpenClaw();
            intake.SlideMotorAutoOut(SlidePose);
            timer.reset();
            if(TurretSide == Turret.TurretHorizontal.AutoLeft){
                state = State.AutoPickAndExchangeLeft;
            }else if(TurretSide == Turret.TurretHorizontal.AutoRight){
                state = State.AutoPickAndExchangeRight;
            }
        }, 200);
//        PauseTimeMilliseconds(750);
//        intake.CloseClaw();
//        PauseTimeMilliseconds(250);
//        AutoMoveToExchange2(TurretSide);
    }
    public void AutoMoveToExchange2(Turret.TurretHorizontal TurretSide){
        intakeScheduler.start();
        intake.CloseClaw();
        turret.SlideOut();
        turret.CloseClaw();
        turret.MoveVertical(Turret.TurretHeight.Default);
        intakeScheduler.schedule(() -> {
            turret.OpenClaw();
            intake.AutoFlipUp();
        }, 150);
        intakeScheduler.schedule(() -> {
            intake.IntakeNewExchange();
            intake.SlideMotorExchange();
        }, 400);
        intakeScheduler.schedule(() -> turret.SlideMid(), 300);
        intakeScheduler.schedule(() -> turret.CloseClaw(), 550);
        intakeScheduler.schedule(() -> intake.OpenClaw(), 50);
        intakeScheduler.schedule(() -> {
            lift.MoveLift(Constants.LiftMid, Constants.HighVelocity);
            intake.IntakeIn();
        }, 50);
        intakeScheduler.schedule(() -> AutoExtake(TurretSide), 500);
    }

    public void AutoExtake(Turret.TurretHorizontal TurretSide) {
        extakeScheduler.start();
        lift.MoveLift(Constants.AutoLiftHigh, Constants.HighVelocity);
        turret.SlideMid();
        extakeScheduler.schedule(() -> turret.MoveVertical(Turret.TurretHeight.CycleVertical), 300);
        extakeScheduler.schedule(() -> turret.MoveHorizontal(TurretSide), 300);
        extakeScheduler.schedule(() -> turret.MoveHorizontal(TurretSide), 300);
        extakeScheduler.schedule(() -> turret.MoveVertical(Turret.TurretHeight.Flipped), 300);
        extakeScheduler.schedule(() -> turret.OpenClaw(), 600);
        extakeScheduler.schedule(() -> turret.MoveVertical(Turret.TurretHeight.CycleVertical), 300);
        extakeScheduler.schedule(() -> {
            turret.CloseClaw();
            turret.MoveHorizontal(Turret.TurretHorizontal.Center);
        }, 100);
        extakeScheduler.schedule(() -> turret.MoveVertical(Turret.TurretHeight.Default), 500);
        extakeScheduler.schedule(() -> lift.MoveLift(Lift.LiftHeight.Default), 400);
        extakeScheduler.schedule(() -> {
            turret.SlideOut();
            turret.OpenClaw();
        }, 700);
    }

    public void AutoReturn() {
        turret.CloseClaw();
        turret.SlideIn();
        intake.AutoFlipUp();
        intake.SlideMotorIn();
//        PauseTimeMilliseconds(500);
    }

    //endregion



    public void PickAndExchange(){
        while(!intake.AutoCloseClaw()){
        intake.AutoCloseClaw();
        if(intake.AutoCloseClaw()) {
            MoveToExchange2();
            state = State.None;
            }
        }
    }

    public void CameraCentering(){
        while(!IntakeReturn()){
            double turretPose;
            turretPose = pipeline.xErrorServo + Constants.TurretDefault;
            turret.MoveHorizontal(turretPose);
        }
    }

    public void Move(Pose2d drivePower) {
        driveTrain.Move(drivePower);
    }

    public void MoveLift(Lift.LiftHeight height) throws Exception {
        lift.MoveLift(height);
    }

    public void MoveLift(int offSet) throws Exception {
        lift.MoveLift(offSet);
    }

    public void IntakeOpenClaw() {
        intake.OpenClaw();
    }

    public void OpenCloseIntakeClaw() {
        intake.OpenCloseClaw();
    }

    public void OpenClaw() {
        turret.OpenClaw();
    }

    public void CloseClaw() {
        turret.CloseClaw();
    }

    public void TurretRight() {
        turret.MoveHorizontal(Turret.TurretHorizontal.Right);
    }

    public void TurretLeft() {
        turret.MoveHorizontal(Turret.TurretHorizontal.Left);
    }

    public void LiftMedium() {
        lift.MoveLift(Lift.LiftHeight.Medium);
    }

    public void ExtakeSlideInOut() {
        if(turret.IsSlideOut())
            turret.SlideIn();
        else
            turret.SlideOut();
    }

    public void MoveLiftOffset(float positionOffset) {
        int pos = (int)positionOffset;
        lift.MoveLift(pos);
    }

    public void MoveIntake(int offset) {
        intake.SetIntakePosition(offset);
    }

    public void MoveSlide(int offset) {
        intake.SetSlidePositionOffset(offset);
    }

    public void StompDown() {
        driveTrain.StompDown();
    }

    public void StompUp() {
        driveTrain.StompUp();
    }

    public void TurretVertical(double offset) {
        turret.MoveVerticalOffset(offset);
    }

    public void TurretHorizontal(double offset) {
        turret.MoveHorizontalOffset(offset);
    }

    public void TurretCenter() {
        turret.MoveHorizontal(Turret.TurretHorizontal.Center);
    }

    public void ToggleClawFlip() {
        intake.ToggleFlip();
    }
    public void IntakeFlipDown(){
        intake.FlipDown();
    }
    public void IntakeFlipUp(){
        intake.FlipUp();
    }
}
