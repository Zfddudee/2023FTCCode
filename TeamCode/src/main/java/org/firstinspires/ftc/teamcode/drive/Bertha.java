package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bertha{

    enum State {
        None,
        PreConePickUp,
        PickAndExchange,
        PickAndExchange_Step2,
        PickAndExchange_Step3,
        PickUpOverRide,
        ExchangeToExtake,
        IntakeReturn,
        MoveToExchange,
        MoveToExchange2
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

    public Bertha(HardwareMap map, Telemetry tel){
        lift = new Lift(map, tel);
        driveTrain = new DriveTrain(map, tel);
        turret = new Turret(map, tel);
        intake = new Intake(map, tel);

        timer = new ElapsedTime();
        state = State.None;
        telemetry = tel;
    }

    private void ResetStartTimer() {
        timer.reset();
        timer.startTime();
    }

    private void PauseTimeMilliseconds(int milliseconds) {
        ResetStartTimer();
        boolean flag = true;
        while(flag)
        {
            if(timer.milliseconds() > milliseconds)
                flag = false;
        }
    }

    public void RunOpMode() {
        telemetry.addData("Current State: ", state);
        switch (state)
        {
            case PreConePickUp:
                if(turret.IsClawOpen() && intake.GetCurrentSlidePosition() > Constants.IntakeExchanging - 100 ) {
                    intake.IntakeLow();
                    intake.FlipDown();
                    state = State.None;
                }
                break;
            case PickAndExchange:
                if(intake.SlowIntakeWheels()){
                    MoveToExchangeWithCone();
                }
                break;
            case PickAndExchange_Step2:
                intake.IntakeOut();
                PauseTimeMilliseconds(300);
                //intake.WaitTillIntakeMotorIsComplete();
                intake.SlideMotorExchange();
                if(!intake.IsSlideMotorBusy() ) {
                    intake.IntakeNewExchange();
                    turret.MoveVertical(Turret.TurretHeight.Default);
                    turret.SlideMid();
                    state = State.PickAndExchange_Step3;
                }
                break;
            case PickAndExchange_Step3:
                if(turret.IsAtVerticalPosition(Constants.TurretDefault, 10)) {
                    turret.CloseClaw();
                    if(turret.IsClawClosed()) {
                        intake.IntakeSpinOut();
                        PauseTimeMilliseconds(150);
                        lift.MoveLift(Lift.LiftHeight.Medium);
                        intake.IntakeSpinStop();
                        state = State.None;
                    }
                }
                break;
            case MoveToExchange:
                this.MoveToExchangeWithCone();
                break;
            default:
                break;
        }

    }

    private void MoveToExchangeWithCone() {
        intake.IntakeLow();
        if(intake.GetIntakePosition() >= Constants.IntakeFlipsLow) {
            intake.FlipUp();
            state = State.PickAndExchange_Step2;
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

    public void WheelsSpinOut() {
        intake.IntakeSpinOut();
    }

    //This moves the intake into a position to grab a cone in its low position
    public void PreConePickUp() {
        if (state != State.PreConePickUp) {
            state = State.PreConePickUp;
            ResetStartTimer();
            turret.MoveVertical(Turret.TurretHeight.Low);
            turret.OpenClaw();
            intake.SlideMotorExchange();
        }
    }

    //This picks up a cone and moves it past the exchange point to where the cone is in possession of controller 2
    public void PickAndExchange() {
        if(state != State.PickAndExchange || state != State.PickAndExchange_Step2 || state != State.PickAndExchange_Step3) {
            state = State.PickAndExchange;
            ResetStartTimer();
            intake.IntakeSpinIn();
            intake.IntakeOut();
            PauseTimeMilliseconds(300);
            intake.IntakeIn();
        }
    }

    public void PickUpOverRide() {
        if(state != State.PickUpOverRide) {
            state = State.PickUpOverRide;
            intake.IntakeSpinStop();
            intake.IntakeLow();
            state = State.None;
        }
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

    public void OpenClaw() {
        turret.OpenClaw();
    }

    public void CloseClaw() {
        turret.CloseClaw();
    }

    public void IntakeReturn() {
        if(state != State.IntakeReturn) {
            if (!turret.IsAtHorizontalPosition(Constants.TurretDefault, 5.0)) {
                turret.MoveHorizontal(Turret.TurretHorizontal.Center);
                PauseTimeMilliseconds(750);
            }
            if(turret.IsSlideOut()) {
                turret.SlideIn();
            }
            if(!turret.IsAtVerticalPosition(Constants.ExtakeFlipIn, 2.0)) {
                turret.MoveVertical(Turret.TurretHeight.Default);
            }
            turret.MoveHorizontal(Turret.TurretHorizontal.Center);
            PauseTimeMilliseconds(250);
            turret.MoveVertical(Turret.TurretHeight.Default);
            lift.MoveLift(Lift.LiftHeight.Default);
            state = State.None;
        }
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

    public void Reset() {
        turret.CloseClaw();
        lift.MoveLift(Lift.LiftHeight.Medium);
//        PauseTimeMilliseconds();
//        lift.WaitTillCompleteMoveLift();
        intake.FlipUp();
        PauseTimeMilliseconds(300);
        intake.IntakeIn();
       PauseTimeMilliseconds(300);
//        intake.WaitTillIntakeMotorIsComplete();
        intake.SlideMotorIn();
        turret.MoveHorizontal(Turret.TurretHorizontal.Center);
        PauseTimeMilliseconds(500);
        turret.SlideIn();
        PauseTimeMilliseconds(500);
        turret.MoveVertical(Turret.TurretHeight.Default);
        PauseTimeMilliseconds(250);
        lift.MoveLift(Lift.LiftHeight.Default);
        turret.OpenClaw();
        state = State.None;
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
    
    public void MoveToExchange(){
        MoveToExchangeWithCone();
        state = State.MoveToExchange;
    }

    public void MoveToExchange2() {
        turret.SlideOut();
        turret.CloseClaw();
        turret.MoveVertical(Turret.TurretHeight.Default);
        PauseTimeMilliseconds(150);
        turret.OpenClaw();
        intake.FlipUp();
        PauseTimeMilliseconds(400);
        intake.IntakeIn();
        intake.SlideMotorExchange();
        PauseTimeMilliseconds(500);
        turret.SlideIn();
        PauseTimeMilliseconds(750);
        intake.IntakeSpinOut();
        PauseTimeMilliseconds(100);
        intake.IntakeSpinStop();
        turret.CloseClaw();
        lift.MoveLift(Lift.LiftHeight.Medium);

    }

    public void TeleOpCycle() {
        lift.MoveLift(Lift.LiftHeight.High);
        turret.MoveVertical(Turret.TurretHeight.CycleVertical);
        PauseTimeMilliseconds(500);
        turret.MoveHorizontal(Turret.TurretHorizontal.CycleHorizontal);
    }
}
