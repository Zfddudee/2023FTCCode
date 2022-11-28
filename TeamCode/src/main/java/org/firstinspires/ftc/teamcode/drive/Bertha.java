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
        IntakeReturn
    };

    ///region Robot objects
    private Lift lift;
    private DriveTrain driveTrain;
    private Turret turret;
    private Intake intake;
    //endregion

    private ElapsedTime timer;
    private State state;

    public Bertha(HardwareMap map, Telemetry tel){
        lift = new Lift(map, tel);
        driveTrain = new DriveTrain(map, tel);
        turret = new Turret(map, tel);
        intake = new Intake(map, tel);

        timer = new ElapsedTime();
        state = State.None;
    }


    private void ResetStartTimer() {
        timer.reset();
        timer.startTime();
    }

    public void Move(Pose2d drivePower) {
       driveTrain.Move(drivePower);
    }

    public void ExtendFlipMotor(){
        //TODO: Implement code to extend flip motor
        

    }
    public void RetractFlipMotor(){
        //TODO: Implement retract flip motor
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

    public void RunOpMode() {
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
                if(intake.SlowIntakeWheels()) {
                    intake.IntakeLow();
                    if(intake.GetIntakePosition() >= Constants.IntakeFlipsLow) {
                        intake.FlipUp();
                        state = State.PickAndExchange_Step2;
                    }
                }
                break;
            case PickAndExchange_Step2:
                intake.SlideMotorExchange();
                if(intake.IsIntakeAtPosition(Constants.IntakeExchanging, 10) ) {
                    turret.MoveVertical(Turret.TurretHeight.Default);
                }
                break;
            case PickAndExchange_Step3:
                if(turret.IsAtVerticalPosition(Constants.TurretDefault, 10)) {
                    turret.CloseClaw();
                    if(turret.IsClawClosed()) {
                        intake.IntakeSpinOut();
                        PauseTimeMilliseconds(200);
                        lift.MoveLift(Lift.LiftHeight.Medium);
                        state = State.None;
                    }
                }
                break;
            default:
                break;
        }

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

    //This moves the intake into a position to grab a cone in its low position
    public void PreConePickUp() {
        if (state != State.PreConePickUp) {
            state = State.PreConePickUp;
            ResetStartTimer();
            turret.MoveVertical(Turret.TurretHeight.Low);
            turret.OpenClaw();
            intake.SlideMotorOut();
        }
    }

    //This picks up a cone and moves it past the exchange point to where the cone is in possession of controller 2
    public void PickAndExchange() {
        if(state != State.PickAndExchange || state != State.PickAndExchange_Step2 || state != State.PickAndExchange_Step3) {
            state = State.PickAndExchange;
            ResetStartTimer();
            intake.IntakeSpinIn();
            intake.IntakeOut();
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
            lift.MoveLift(Lift.LiftHeight.Default);
            state = State.None;
        }
    }
}
