package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bertha{

    enum State {
        None,
        PreConePickUp
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

    public void Execute() {
        switch (state)
        {
            case PreConePickUp:
                if(turret.IsClawOpen() && intake.GetCurrentSlidePosition() > Constants.IntakeExchanging - 100 ) {
                    intake.IntakeLow();
                    intake.FlipDown();
                    state = State.None;
                }
                break;
            default:
                break;
        }

    }

    public void PreConePickUp() {
        ResetStartTimer();
        turret.MoveVertical(Turret.TurretHeight.Low);
        turret.OpenClaw();
        intake.SlideMotorOut();
        state = State.PreConePickUp;
    }
    private void ResetStartTimer() {
        timer.reset();
        timer.startTime();
    }
}
