package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.JunctionPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.JunctionPipeline2;

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
    }
    enum Intaking {
        None,
        TurretSlideOut,
        IntakeFlip,
        IntakeSlide,
        AutoCloseClaw,
        SlideIn,
        Flipin,
        SlideFullIn,
        ExhchangeToExtake,
        Reset
    }
    enum Extaking {
        None,
        Exchanging,
        LiftUpFlipOutSlideOut,
        TurretTurnLeft,
        TurretTurnRight,
        TurretAutoTurn,
        ClawDrop,
        TurretCenter,
        Returning,
        Reset
    }

    ///region Robot objects
    private Lift lift;
    private DriveTrain driveTrain;
    private Turret turret;
    private Intake intake;
    //endregion

    private ElapsedTime timer;
    private State state;
    private Intaking intaking;
    private Extaking extaking;
    private Telemetry telemetry;

    private boolean IntakeGo = false;

    double CameraXError;
    double Distance;
    double DistanceError;
    double turretPose;
    double slidePose1 = Constants.SlideIn;
    double slidePose2 = Constants.SlideIn2;
    public Bertha(HardwareMap map, Telemetry tel){
        lift = new Lift(map, tel);
        driveTrain = new DriveTrain(map, tel);
        turret = new Turret(map, tel);
        intake = new Intake(map, tel);

        timer = new ElapsedTime();
        state = State.None;
        intaking = Intaking.None;
        extaking = Extaking.None;
        telemetry = tel;
    }

    private void LogAllTelemetry(){
        telemetry.addData("Current State: ", state);
        telemetry.addData("X val 1", pipeline.x);
        telemetry.addData("X val 2", pipeline2.x);
        telemetry.addData("X val T1", berthaTeleOp.X1);
        telemetry.addData("X val T2", berthaTeleOp.X2);
        turret.Telemetry();
        intake.Telemetry();
        lift.Telemetry();
    }

//    HardwareMap hardwareMap;
    //region TeleOp
    JunctionPipeline pipeline;
    JunctionPipeline2 pipeline2;
    BerthaTeleOp berthaTeleOp;
    public void RunOpMode() {
        pipeline = new JunctionPipeline();
        pipeline2 = new JunctionPipeline2();
        berthaTeleOp = new BerthaTeleOp();

//Intaking cases
        //todo
        // make it so intake can be brought out without full intake process
        // and make it so returning works then test code.
        // Make possibility to exchange without sensor.
        //Todo
        // For full retract extake slide all the way out
        // turret center and lift down then move extake arm to hover
        // then move intake fully in and arm down and extake in
        switch (intaking) {
            case TurretSlideOut:
                turret.SlideOut();
                turret.MoveVertical(Turret.TurretHeight.Low);
                intake.FlipDown();
                if (timer.milliseconds() >= 500)
                    intaking = Intaking.IntakeFlip;
                break;
            case IntakeFlip:
                intake.IntakeOut();
                intake.FlipDown();
                if(intake.IsIntakeFlipAtPosition(Constants.IntakeFlips, 250))
                    intake.OpenClaw();
                if(IntakeGo && intake.IsIntakeFlipAtPosition(Constants.IntakeFlips, 50)) {
//                    IntakeGo = false;
                    intaking = Intaking.IntakeSlide;
                }
                break;
            case IntakeSlide:
                intake.OpenClaw();
                intake.SlideMotorOut();
                intake.AutoCloseClaw();
                if(intake.AutoCloseClaw()) {
                    intaking = Intaking.SlideIn;
                }

                break;
            case SlideIn:
                lift.MoveLift(Lift.LiftHeight.Default);
                intake.CloseClaw();
                turret.SlideOut();
                intake.FlipUp();
                turret.MoveVertical(Turret.TurretHeight.Default);
                intake.SlideMotorWall();
//                if(!intake.AutoCloseClaw()) {
//                    intaking = Intaking.IntakeFlip;
//                }
                if(intake.IsIntakeSlideAtPosition(Constants.IntakeWall, 50))
                    intaking = Intaking.Flipin;
                break;
            case Flipin:
                intake.SlideMotorExchange();
                intake.IntakeNewExchange();
                if(!intake.AutoCloseClaw()) {
                    intaking = Intaking.IntakeFlip;
                }
                else if(intake.IsIntakeFlipAtPosition(Constants.IntakeNewExchange, 50)) {
                    timer.reset();
                    intaking = Intaking.ExhchangeToExtake;
                }
                break;
            case ExhchangeToExtake:
                turret.SlideMid();
                if(timer.milliseconds() >= 500){
                    turret.CloseClaw();
                }
                if(timer.milliseconds() >= 550){
                    intake.OpenClaw();
                    extaking = Extaking.Exchanging;
                    intaking = Intaking.None;
                }
                break;

//This is the case that starts the reset
            case Reset:
                intake.CloseClaw();
                intake.FlipUp();
                if(lift.LiftPosition() >= 400){
                    intake.IntakeIn();
                    intake.SlideMotorIn();
                    turret.MoveHorizontal(Turret.TurretHorizontal.Center);
                    intaking = Intaking.None;
                }
                break;

        }
//Extaking cases
        switch (extaking){
            case Exchanging:
                lift.MoveLift(Lift.LiftHeight.High);
                intake.IntakeIn();
                turret.SlideIn();
                turret.MoveVertical(Turret.TurretHeight.CycleVertical);
                if(lift.IsLiftAtPosition(Constants.LiftHigh, 200)) {
                    extaking = Extaking.TurretAutoTurn;
                    turret.MoveVertical(Turret.TurretHeight.Flipped);
                }
                break;
            case TurretTurnLeft:
                turret.MoveHorizontal(Constants.TurretLeft);
                break;
            case TurretTurnRight:
                turret.MoveHorizontal(Constants.TurretRight);
                break;
            case TurretAutoTurn:
                turretPose = Constants.TurretLeft;
                state = State.CameraCentering;
                break;
                //ClawOpening then moving to returning
            case ClawDrop:
                turret.OpenClaw();
                state = State.None;
                if(timer.time() >= 100)
                    turret.MoveVertical(Turret.TurretHeight.CycleVertical);
                if(timer.time() >= 300) {
                    timer.reset();
                    extaking = Extaking.TurretCenter;
                }
                break;
            case TurretCenter:
                turret.MoveHorizontal(Constants.TurretDefault);
                if(timer.time() >= 500)
                    extaking = Extaking.Returning;
                break;
            case Returning:
                turret.MoveVertical(Turret.TurretHeight.Default);
                lift.MoveLift(Lift.LiftHeight.Default);
                turret.SlideOut();
                turret.OpenClaw();
                extaking = Extaking.None;
                break;


//This is the case that starts the reset
            case Reset:
                turret.CloseClaw();
                lift.MoveLift(Lift.LiftHeight.Medium);
                turret.SlideOut();
                if(intake.IsIntakeFlipAtPosition(0, 80)){
                    turret.SlideIn();
                    turret.MoveVertical(Turret.TurretHeight.Default);
                    lift.MoveLift(Lift.LiftHeight.Default);
                    turret.OpenClaw();
                    extaking = Extaking.None;
                }
                break;

        }
        switch (state)
        {
            case PickAndExchange:
                intake.AutoCloseClaw();
                if(intake.AutoCloseClaw()){
                    MoveToExchange2();
                    state = State.None;
                }
                break;
//
            case CameraCentering:
//                0.006 points per degree
                //When too far left CameraXError is positive
                CameraXError = (pipeline.x - pipeline2.x);
                Distance = (45216114153.52 *  Math.pow(((pipeline.x + pipeline2.x)/2), -3.87))/2.54;
                DistanceError = Distance - 13;
                if(CameraXError >= 50 && pipeline.x != 0)
                    turretPose = turretPose + Constants.TurretStepOver;
                else if(CameraXError <= -50 && pipeline.x != 0)
                    turretPose = turretPose - Constants.TurretStepOver;
                if(DistanceError >= 0.5 && pipeline.x != 0) {
                    slidePose1 = slidePose1 - 0.1;
                    slidePose1 = slidePose1 + 0.1;
                }
                else if(DistanceError <= 0.5 && pipeline.x != 0) {
                    slidePose1 = slidePose1 + 0.1;
                    slidePose1 = slidePose1 - 0.1;
                }
                //                turretPose = pipeline.xErrorServo + Constants.TurretDefault;
                turret.SetSlidePosition(slidePose1, slidePose2);
                turret.MoveHorizontal(turretPose);
                break;

            default:
                break;
        }
        this.LogAllTelemetry();
    }

    public void CameraCenterTest(){
        turretPose = Constants.TurretLeft;
        turret.MoveVertical(Turret.TurretHeight.CycleVertical);
        state = State.CameraCentering;
    }
    public void PreConePickup(){
        timer.reset();
        IntakeGo = true;
        intaking = Intaking.TurretSlideOut;
    }

    public void MoveToExchange2() {
        intaking = Intaking.SlideIn;
    }

    public void PickUpOverRide() {
        //TODO Fix and re add in to code
//        turret.SlideOut();
//        intakeScheduler.schedule(() -> {
//            turret.MoveVertical(Turret.TurretHeight.Low);
//            intake.FlipDown();
//    }, 500);
//        intakeScheduler.schedule(() -> intake.IntakeOut(), 300);
//        intakeScheduler.schedule(() -> intake.OpenClaw(), 500);
//        intakeScheduler.schedule(() -> {
//            intake.OpenClaw();
//            intake.SlideMotorOut();
//        }, 100);
    }

    /**
     * B button press on gamepad 2 that brings lift down
     */
    public void IntakeReturn() {
        extaking = Extaking.ClawDrop;
    }

    /**
     * Share button function that brings everything back.
     */
    public void Reset() {
        extaking = Extaking.Reset;
        intaking = Intaking.Reset;
    }

    public void TeleOpCycle() {
        extaking = Extaking.Exchanging;
    }

    public void Move(Pose2d drivePower) {
        driveTrain.Move(drivePower);
    }

    public void IntakeOpenClaw() {
        intake.OpenClaw();
    }

    public void OpenCloseIntakeClaw() {
        intake.OpenCloseClaw();
    }

    public void OpenClaw() {
        if(extaking == Extaking.TurretAutoTurn || extaking == Extaking.TurretTurnLeft || extaking == Extaking.TurretTurnRight)
        extaking = Extaking.ClawDrop;
        else turret.OpenClaw();
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

    public void MoveIntake(int offset) {
        intake.SetIntakeFlipPosition(offset);
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
}
