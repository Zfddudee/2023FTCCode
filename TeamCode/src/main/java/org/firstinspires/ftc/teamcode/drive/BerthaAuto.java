package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.JunctionPipeline;

public class BerthaAuto extends Bertha {
    public BerthaAuto (HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
    }

        public void AutoCheck() {
        turret.CloseClaw();
        turret.SlideIn();
    }

    public void PlaceConeOverJunction() {
        turret.MoveHorizontal(Turret.TurretHorizontal.AutoLeft);
        turret.MoveVertical(Turret.TurretHeight.Flipped);
    }
}

//@Config
//public class BerthaAuto {
//
//    enum State {
//        None,
//        PickAndExchange,
//        AutoPickAndExchangeRight,
//        AutoPickAndExchangeLeft,
//        CameraCentering,
//        ExchangeToExtake,
//        IntakeReturn,
//    }
//    enum Intaking {
//        TurretSlideOut,
//        IntakeFlip,
//        IntakeSlide,
//        AutoCloseClaw,
//        SlideIn,
//        Flipin,
//        SlideFullIn,
//        ExhchangeToExtake
//    }
//    enum Extaking {
//        Exchanging,
//        LiftUpFlipOutSlideOut,
//        TurretTurnLeft,
//        TurretTurnRight,
//        TurretAutoTurn,
//        ClawDrop,
//        TurretCenter,
//        Returning
//    }
//
//    ///region Robot objects
//    private Lift lift;
//    private DriveTrain driveTrain;
//    private Turret turret;
//    private Intake intake;
//    //endregion
//
//    private ElapsedTime timer;
//    private State state;
//    private Intaking intaking;
//    private Extaking extaking;
//    private Telemetry telemetry;
//       public BerthaAuto(HardwareMap map, Telemetry tel){
//        lift = new Lift(map, tel);
//        driveTrain = new DriveTrain(map, tel);
//        turret = new Turret(map, tel);
//        intake = new Intake(map, tel);
//
//        timer = new ElapsedTime();
//        state = State.None;
//        telemetry = tel;
//    }
//
//    private void LogAllTelemetry(){
//        telemetry.addData("Current State: ", state);
//        turret.Telemetry();
//        intake.Telemetry();
//        lift.Telemetry();
//    }
//
////    HardwareMap hardwareMap;
//    //region TeleOp
//    JunctionPipeline pipeline;
//    public void RunOpMode() {
////Intaking cases
//        //todo make it so if it does not pick up cone it goes back and make it so returning works then test code.
//        switch (intaking) {
//            case TurretSlideOut:
//                turret.SlideMid();
//                turret.MoveVertical(Turret.TurretHeight.Low);
//                intake.FlipDown();
//                if (timer.time() >= 500)
//                    intaking = Intaking.IntakeFlip;
//                break;
//            case IntakeFlip:
//                intake.IntakeOut();
//                intake.FlipDown();
//                if(intake.IsIntakeFlipAtPosition(Constants.IntakeFlips, 250))
//                    intake.OpenClaw();
//                if(intake.IsIntakeFlipAtPosition(Constants.IntakeFlips, 50))
//                    intaking = Intaking.IntakeSlide;
//
//                break;
//            case IntakeSlide:
//                intake.OpenClaw();
//                intake.SlideMotorOut();
//                intake.AutoCloseClaw();
//                if(intake.AutoCloseClaw()) {
//                    intaking = Intaking.SlideIn;
//                }
//
//                break;
//            case SlideIn:
//                lift.MoveLift(Lift.LiftHeight.Default);
//                intake.CloseClaw();
//                turret.SlideOut();
//                turret.CloseClaw();
//                intake.FlipUp();
//                turret.MoveVertical(Turret.TurretHeight.Default);
//                intake.SlideMotorWall();
//                if(!intake.AutoCloseClaw()) {
//                    intaking = Intaking.IntakeFlip;
//                }
//                else if(intake.IsIntakeSlideAtPosition(Constants.IntakeWall, 50))
//                    intaking = Intaking.Flipin;
//                break;
//            case Flipin:
//                intake.SlideMotorExchange();
//                intake.IntakeNewExchange();
//                if(!intake.AutoCloseClaw()) {
//                    intaking = Intaking.IntakeFlip;
//                }
//                else if(intake.IsIntakeSlideAtPosition(Constants.IntakeNewExchange, 50)) {
//                    timer.reset();
//                    intaking = Intaking.ExhchangeToExtake;
//                }
//                break;
//            case ExhchangeToExtake:
//                turret.SlideMid();
//                if(timer.time() >= 500){
//                    turret.CloseClaw();
//                }
//                if(timer.time() >= 550){
//                    intake.OpenClaw();
//                    extaking = Extaking.Exchanging;
//                }
//                break;
//        }
////Extaking cases
//        switch (extaking){
//            case Exchanging:
//                lift.MoveLift(Lift.LiftHeight.High);
//                intake.IntakeIn();
//                turret.SlideIn();
//                turret.MoveVertical(Turret.TurretHeight.CycleVertical);
//                if(lift.IsLiftAtPosition(Constants.LiftHigh, 200)) {
//                    extaking = Extaking.TurretAutoTurn;
//                    turret.MoveVertical(Turret.TurretHeight.Flipped);
//                }
//                break;
//            case TurretTurnLeft:
//                turret.MoveHorizontal(Constants.TurretLeft);
//                break;
//            case TurretTurnRight:
//                turret.MoveHorizontal(Constants.TurretRight);
//                break;
//            case TurretAutoTurn:
//                turret.MoveHorizontal(Constants.TurretLeft);
//                state = State.CameraCentering;
//                break;
//                //ClawOpening then moving to returning
//            case ClawDrop:
//                turret.OpenClaw();
//                if(timer.time() >= 100)
//                    turret.MoveVertical(Turret.TurretHeight.CycleVertical);
//                if(timer.time() >= 300) {
//                    timer.reset();
//                    extaking = Extaking.TurretCenter;
//                }
//                break;
//            case TurretCenter:
//                turret.MoveHorizontal(Constants.TurretDefault);
//                if(timer.time() >= 500)
//                    extaking = Extaking.Returning;
//                break;
//            case Returning:
//                turret.MoveVertical(Turret.TurretHeight.Default);
//                lift.MoveLift(Lift.LiftHeight.Default);
//                turret.SlideOut();
//                turret.OpenClaw();
//                break;
//        }
//        switch (state)
//        {
//            case AutoPickAndExchangeRight:
//                intake.AutoCloseClaw();
//                if(intake.AutoCloseClaw() || timer.milliseconds() >= 750){
//                    AutoMoveToExchange2(Turret.TurretHorizontal.AutoRight);
//                    state = State.None;
//                }
//                break;
//            case AutoPickAndExchangeLeft:
//                intake.AutoCloseClaw();
//                if(intake.AutoCloseClaw() || timer.milliseconds() >= 750){
//                    AutoMoveToExchange2(Turret.TurretHorizontal.AutoLeft);
//                    state = State.None;
//                }
//                break;
//            case CameraCentering:
////                0.006 points per degree
//                double turretPose;
//                turretPose = pipeline.xErrorServo + Constants.TurretDefault;
//                turret.MoveHorizontal(turretPose);
//                break;
//
//            default:
//                break;
//        }
//        this.LogAllTelemetry();
//    }
//
//    //region Autonomous
//    public void AutoCheck() {
//        turret.CloseClaw();
//        turret.SlideIn();
//    }
//
//    public void AutoIntake(int ConePosition, int SlidePose, Turret.TurretHorizontal TurretSide) {
//        turret.SlideOut();
////        intakeScheduler.schedule(() -> {
////            turret.MoveVertical(Turret.TurretHeight.Low);
////            intake.FlipDown();
////        }, 500);
////        intakeScheduler.schedule(() -> intake.AutoIntakeOut(ConePosition), 300);
////        intakeScheduler.schedule(() -> intake.OpenClaw(), 200);
////        intakeScheduler.schedule(() -> intake.OpenClaw(), 100);
////        intakeScheduler.schedule(() -> {
//            intake.OpenClaw();
//            intake.SlideMotorAutoOut(SlidePose);
//            timer.reset();
//            if(TurretSide == Turret.TurretHorizontal.AutoLeft){
//                state = State.AutoPickAndExchangeLeft;
//            }else if(TurretSide == Turret.TurretHorizontal.AutoRight){
//                state = State.AutoPickAndExchangeRight;
//            }
////        }, 200);
////        PauseTimeMilliseconds(750);
////        intake.CloseClaw();
////        PauseTimeMilliseconds(250);
////        AutoMoveToExchange2(TurretSide);
//    }
//    public void AutoMoveToExchange2(Turret.TurretHorizontal TurretSide){
////        intakeScheduler.start();
//        intake.CloseClaw();
//        turret.SlideOut();
//        turret.CloseClaw();
//        turret.MoveVertical(Turret.TurretHeight.Default);
////        intakeScheduler.schedule(() -> {
////            turret.OpenClaw();
////            intake.AutoFlipUp();
////        }, 150);
//////        intakeScheduler.schedule(() -> {
////            intake.IntakeNewExchange();
////            intake.SlideMotorExchange();
////        }, 400);
////        intakeScheduler.schedule(() -> turret.SlideMid(), 300);
////        intakeScheduler.schedule(() -> turret.CloseClaw(), 550);
////        intakeScheduler.schedule(() -> intake.OpenClaw(), 50);
////        intakeScheduler.schedule(() -> {
////            lift.MoveLift(Constants.LiftMid);
////            intake.IntakeIn();
////        }, 50);
////        intakeScheduler.schedule(() -> AutoExtake(TurretSide), 500);
//    }
//
//    public void AutoExtake(Turret.TurretHorizontal TurretSide) {
//        //Todo Make code for auto again
////        lift.MoveLift(Constants.AutoLiftHigh, Constants.HighVelocity);
////        turret.SlideMid();
////        extakeScheduler.schedule(() -> turret.MoveVertical(Turret.TurretHeight.CycleVertical), 300);
////        extakeScheduler.schedule(() -> turret.MoveHorizontal(TurretSide), 300);
////        extakeScheduler.schedule(() -> turret.MoveHorizontal(TurretSide), 300);
////        extakeScheduler.schedule(() -> turret.MoveVertical(Turret.TurretHeight.Flipped), 300);
////        extakeScheduler.schedule(() -> turret.OpenClaw(), 600);
////        extakeScheduler.schedule(() -> turret.MoveVertical(Turret.TurretHeight.CycleVertical), 300);
////        extakeScheduler.schedule(() -> {
////            turret.CloseClaw();
////            turret.MoveHorizontal(Turret.TurretHorizontal.Center);
////        }, 100);
////        extakeScheduler.schedule(() -> turret.MoveVertical(Turret.TurretHeight.Default), 500);
////        extakeScheduler.schedule(() -> lift.MoveLift(Lift.LiftHeight.Default), 400);
////        extakeScheduler.schedule(() -> {
////            turret.SlideOut();
////            turret.OpenClaw();
////        }, 700);
//    }
//
//    public void AutoReturn() {
//        turret.CloseClaw();
//        turret.SlideIn();
//        intake.AutoFlipUp();
//        intake.SlideMotorIn();
////        PauseTimeMilliseconds(500);
//    }
//}
