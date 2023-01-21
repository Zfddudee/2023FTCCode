package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.JunctionPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.JunctionPipeline2;

import java.time.Clock;

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
        IntakeFlipAuto,
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
    protected Lift lift;
    protected DriveTrain driveTrain;
    protected Turret turret;
    protected Intake intake;
    //endregion

    ///region Private members
    private ElapsedTime timer;
    private ElapsedTime CurrentTime;
    private State state;
    protected Intaking intaking;
    private Extaking extaking;
    private Telemetry telemetry;

    private boolean IntakeGo = false;
    private Lift.LiftHeight LiftHeight;
    private int LiftPosition = 3;

    double LastTime = 0;
    double Dt;
    double Kd = 0.5;
    double Kp = 0.5;
    double CameraXError;
    double Distance;
    double DistanceError;
    double DistanceErrorLast = 0;
    double turretPose = Constants.TurretHorizontalCycle;
    double slidePose1 = Constants.SlideIn;
    double slidePose2 = Constants.SlideIn2;

    protected int intakeHeightOffset = 0;
    ///endregion

    public Bertha(HardwareMap map, Telemetry tel){
        lift = new Lift(map, tel);
        driveTrain = new DriveTrain(map, tel);
        turret = new Turret(map, tel);
        intake = new Intake(map, tel);

        timer = new ElapsedTime();
        CurrentTime = new ElapsedTime();
        state = State.None;
        intaking = Intaking.None;
        extaking = Extaking.None;
        telemetry = tel;
    }

    private void LogAllTelemetry(){
        telemetry.addData("Current State: ", state);
        telemetry.addData("X val 1", pipeline.x);
        telemetry.addData("X val 2", pipeline2.x);
        telemetry.addData("Distance", Distance);
        telemetry.addData("Distance Error", DistanceError);
        telemetry.addData("X val T1", Constants.X1);
        telemetry.addData("X val T2", Constants.X2);
        telemetry.addData("TurretStepover: ", Constants.TurretStepOver);
        telemetry.addData("SlideStepover: ", Constants.SlideStepover);
        telemetry.addData("SlidePos: ", slidePose1);
        telemetry.addData("SlidePos2: ", slidePose2);
        telemetry.addData("TurretPos: ", turretPose);
        turret.Telemetry();
        intake.Telemetry();
        lift.Telemetry();
    }
    //region TeleOp
    JunctionPipeline pipeline;
    JunctionPipeline2 pipeline2;
    public void RunOpMode() {
        pipeline = new JunctionPipeline();
        pipeline2 = new JunctionPipeline2();
        CurrentTime.reset();
//Intaking cases
        //todo
        // make it so intake can be brought out without full intake process
        // and make it so returning works then test code.
        // Make possibility to exchange without sensor.

        //Todo
        // Make possible to press button to just return intake when driving around
        switch (intaking) {
            case TurretSlideOut:
                if(intake.IsIntakeFlipAtPosition(Constants.IntakeFlips, 150))
                    intaking = Intaking.IntakeSlide;
                else {
                    turret.SlideOut();
                    turret.MoveVertical(Turret.TurretHeight.Low);
                    intake.FlipDown();
                    if (timer.milliseconds() >= 250)
                        intaking = Intaking.IntakeFlip;
                }
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
            case IntakeFlipAuto:
                intake.IntakeOut();
                intake.SetIntakeFlipPosition(intakeHeightOffset);

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
                    timer.reset();
                }

                break;
            case SlideIn:
                lift.MoveLift(Lift.LiftHeight.Default);
                intake.CloseClaw();
            if(timer.milliseconds() >= 100) {
                turret.SlideOut();
                intake.FlipUp();
                turret.MoveVertical(Turret.TurretHeight.Default);
                intake.SlideMotorWall();
//                if(!intake.AutoCloseClaw()) {
//                    intaking = Intaking.IntakeFlip;
//                }
                if (intake.IsIntakeSlideAtPosition(Constants.IntakeWall, 50))
                    intaking = Intaking.Flipin;
            }
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
                turret.OpenClaw();
                if(timer.milliseconds() >= 300){
                    turret.CloseClaw();
                }
                if(timer.milliseconds() >= 400){
                    intake.OpenClaw();
                    extaking = Extaking.Exchanging;
                    intaking = Intaking.None;
                }
                break;

//This is the case that starts the reset
            case Reset:
                state = State.None;
                intake.CloseClaw();
                intake.FlipUp();
                intake.SlideMotorOut();
                if(intake.IsIntakeSlideAtPosition(-650, 80)) {
                    intake.IntakeIn();
                    if (intake.IsIntakeFlipAtPosition(0, 80)) {
                        intake.SlideMotorIn();
                        turret.MoveHorizontal(Turret.TurretHorizontal.Center);
                        intaking = Intaking.None;
                    }
                }
                break;

        }
//Extaking cases
        switch (extaking){
/**
 * Case that finishes the exchange from intake to extake
  */
            case Exchanging:
                LiftPosition = 3;
                turret.MoveVertical(Turret.TurretHeight.CycleVertical);
                lift.MoveLift(Lift.LiftHeight.High);
                intake.IntakeIn();
                turret.SlideIn();
                intake.FlipDown();
                if(lift.IsLiftAtPosition(Constants.LiftMid, 200)) {
                    lift.MoveLift(Lift.LiftHeight.High);
//                    extaking = Extaking.TurretAutoTurn;
                    extaking = Extaking.TurretTurnLeft;
//                    turret.MoveVertical(Turret.TurretHeight.Flipped);
                }
                break;
//Case that turns turret left
            case TurretTurnLeft:
                intake.IntakeOut();
                intake.FlipDown();
                intake.OpenClaw();
                turret.MoveHorizontal(Constants.TurretHorizontalCycle);
                extaking = Extaking.None;
                break;
//Case that turns turret right
            case TurretTurnRight:
                intake.IntakeOut();
                intake.FlipDown();
                intake.OpenClaw();
                turret.MoveHorizontal(Constants.TurretRight);
                extaking = Extaking.None;
                break;
//Case that automatically turns the turret to junction off camera
            case TurretAutoTurn:
                intake.IntakeOut();
                intake.FlipDown();
                intake.OpenClaw();
                turretPose = Constants.TurretHorizontalCycle;
                slidePose1 = Constants.SlideIn;
                slidePose2 = Constants.SlideIn2;
                state = State.CameraCentering;
                extaking = Extaking.None;
                break;
//ClawOpening then moving to returning
            case ClawDrop:
                turret.OpenClaw();
                intake.OpenClaw();
                state = State.None;
                intaking = Intaking.TurretSlideOut;
                if(timer.milliseconds() >= 100)
                    turret.MoveVertical(Turret.TurretHeight.CycleVertical);
                if(timer.milliseconds() >= 300) {
                    timer.reset();
                    extaking = Extaking.TurretCenter;
                }
                break;
//Case that centers the turret
            case TurretCenter:
                turret.MoveHorizontal(Constants.TurretDefault);
                if(timer.milliseconds() >= 500)
                    extaking = Extaking.Returning;
                break;
//Case that brings lift down and final stages of retracting
            case Returning:
                turret.MoveVertical(Turret.TurretHeight.Default);
                lift.MoveLift(Lift.LiftHeight.Default);
                turret.SlideOut();
                turret.OpenClaw();
                extaking = Extaking.None;
                break;


//This is the case that starts the reset
            case Reset:
                turret.MoveVertical(Turret.TurretHeight.CycleVertical);
                turret.OpenClaw();
                lift.MoveLift(Lift.LiftHeight.Medium);
                turret.MoveHorizontal(Turret.TurretHorizontal.Center);
                turret.SlideOut();
                if(intake.IsIntakeFlipAtPosition(0, 80) && timer.milliseconds() >= 200){
                    turret.CloseClaw();
                    turret.SlideIn();
                    turret.MoveVertical(Turret.TurretHeight.Default);
                    lift.MoveLift(Lift.LiftHeight.Default);
                    extaking = Extaking.None;
                }
                break;

        }
        switch (state)
        {
            //Case to start the auto intake
            case PickAndExchange:
                intake.AutoCloseClaw();
                if(intake.AutoCloseClaw()){
                    MoveToExchange2();
                    state = State.None;
                }
                break;
//Case to use the camera to align to a junction
            case CameraCentering:
//                0.006 points per degree
                //When too far left CameraXError is positive
                CameraXError = (Constants.X1 - Constants.X2); //Gets error off center of camera
                Distance = ((15034772.69 *  Math.pow(((Constants.X1 + Constants.X2)/2), -2.46))-1.2)/2.54; //Gets distance from junction
                DistanceError = Distance - 13; //Gets error of distance in inches off of 13 inches

                Dt = CurrentTime.milliseconds() - LastTime;
                LastTime = CurrentTime.milliseconds();



                if(Math.abs(CameraXError) >= 50){
                    Constants.TurretStepOver = CameraXError/-10000;
                    turretPose = turretPose + Constants.TurretStepOver;
                }

                if(Math.abs(DistanceError) >= 1.25){
//                    Constants.SlideStepover = Math.pow((DistanceError), 1.1423) * 0.0025;

                    Constants.SlideStepover = (DistanceError * 0.0025)*Kp - ((DistanceError-DistanceErrorLast)/Dt)*Kd;
                    DistanceErrorLast = DistanceError;

                    slidePose1 = slidePose1 + Constants.SlideStepover;
                    slidePose2 = slidePose2 - Constants.SlideStepover;
                }

//                if(CameraXError >= 50 && Constants.X1 != 0)
//                    turretPose = turretPose + Constants.TurretStepOver;
//                else if(CameraXError <= -50 && Constants.X1 != 0)
//                    turretPose = turretPose - Constants.TurretStepOver;
//
//                if(DistanceError >= Constants.SlideRange && Constants.X1 != 0) {
//                    slidePose1 = slidePose1 + Constants.SlideStepover;
//                    slidePose2 = slidePose2 - Constants.SlideStepover;
//                }
//                else if(DistanceError <= Constants.SlideRange && Constants.X1 != 0) {
//                    slidePose1 = slidePose1 - Constants.SlideStepover;
//                    slidePose2 = slidePose2 + Constants.SlideStepover;
//                }


                if(slidePose1 > Constants.SlideOut)
                    slidePose1 = Constants.SlideOut;
                else if(slidePose1 < Constants.SlideIn)
                    slidePose1 = Constants.SlideIn;
                if(slidePose2 > Constants.SlideIn2)
                    slidePose2 = Constants.SlideIn2;
                else if(slidePose2 < Constants.SlideOut2)
                    slidePose2 = Constants.SlideOut2;
                if(turretPose > Constants.TurretLeft)
                    turretPose = Constants.TurretLeft;
                else if(turretPose < Constants.TurretRight)
                    turretPose = Constants.TurretRight;

//                                turretPose = pipeline.xErrorServo + Constants.TurretDefault;
                turret.SetSlidePosition(slidePose1, slidePose2);
                turret.MoveHorizontal(turretPose);
                break;

            default:
                break;
        }
        this.LogAllTelemetry();
    }

    /**
     * Start intaking process
     */
    public void PreConePickup(){
        timer.reset();
        IntakeGo = true;
        intaking = Intaking.TurretSlideOut;
    }

    /**
     * Exchange from intake to extake
     */
    public void MoveToExchange2() {
        timer.reset();
        intaking = Intaking.SlideIn;
    }

    /**
     * Start intaking process but without distance sensor
     */
    public void PickUpOverRide() {
        //TODO Fix and re add in to code
        intaking = Intaking.None;
        intake.SlideMotorIn();
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
        timer.reset();
        extaking = Extaking.ClawDrop;
    }

    /**
     * Share button function that brings everything back.
     */
    public void Reset() {
        timer.reset();
        extaking = Extaking.Reset;
        intaking = Intaking.Reset;
    }

    /**
     * Moves lift up into cycle position
     */
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
        if(lift.LiftPosition() > 200){
            timer.reset();
        extaking = Extaking.ClawDrop;
        } else turret.OpenClaw();
    }

    public void CloseClaw() {
        turret.CloseClaw();
    }

    public void TurretRight() {
        state = State.None;
        turret.MoveHorizontal(Turret.TurretHorizontal.Right);
    }

    public void TurretLeft() {
        state = State.None;
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
    public void LiftPositioning(int Pos){
        if(LiftPosition > 3)
            Pos = 3;
        if(LiftPosition < 0)
            Pos = 0;
        LiftPosition = LiftPosition + Pos;
        if(LiftPosition == 0)
            LiftHeight = Lift.LiftHeight.Default;
        else if(LiftPosition == 1)
            LiftHeight = Lift.LiftHeight.Low;
        else if(LiftPosition == 2)
            LiftHeight = Lift.LiftHeight.Medium;
        else if(LiftPosition == 3)
            LiftHeight = Lift.LiftHeight.High;

        lift.MoveLift(LiftHeight);
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
        state = State.None;
        turret.MoveHorizontalOffset(offset);
    }

    public void TurretCenter() {
        state = State.None;
        turret.MoveHorizontal(Turret.TurretHorizontal.Center);
    }

    public void ToggleClawFlip() {
        intake.ToggleFlip();
    }
    public void IntakeFlipDown(){
        intake.FlipDown();
    }
}
