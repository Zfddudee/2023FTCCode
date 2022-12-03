package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Turret extends BaseRobot{


    public enum TurretHeight{
        Default,
        Low,
        Flipped,
        CycleVertical
    }

    public enum TurretHorizontal{
        Left,
        Right,
        Center,
        CycleHorizontal,
        AutoRight
    }

    private double currentTurretHeight;
    private double currentTurretHorizontal;
    private ElapsedTime timer;

    private Servo ExtakeFlip1, ExtakeFlip2, Turret1, Claw, SlideExtension, SlideExtension2;

    public Turret(HardwareMap map, Telemetry tel) {
        super(map, tel);
        currentTurretHeight = 0.0;
        currentTurretHorizontal = Constants.TurretDefault;
        timer = new ElapsedTime();
    }

    @Override
    protected void MapHardware() {
        ExtakeFlip1 = hardwareMap.get(Servo.class, "ExtakeFlip1");
        ExtakeFlip2 = hardwareMap.get(Servo.class, "ExtakeFlip2");
        Turret1 = hardwareMap.get(Servo.class, "Turret1");
        Claw = hardwareMap.get(Servo.class, "Claw");
        SlideExtension = hardwareMap.get(Servo.class, "SlideExtension");
        SlideExtension2 = hardwareMap.get(Servo.class, "SlideExtension2");
        Turret1.scaleRange(0.1, 1);
    }

    private double CheckBoundries(double position) {
        if(position > 1)
            position = 1.0;
        if(position < 0)
            position = 0.0;
        return position;
    }

    public void MoveVerticalOffset(double offset) {
        //TODO check these values

        double exFlip1Position = ExtakeFlip1.getPosition() + offset;
        double exFlip2Position = ExtakeFlip2.getPosition() - offset;
        MoveVertical(exFlip1Position, exFlip2Position);
    }

    public void MoveVertical(double extakeFlip1Position, double extakeFlip2Position) {
        //we are going from extended to retracted and we are going back to retracted so center turret
        if( ExtakeFlip1.getPosition() < extakeFlip1Position && extakeFlip1Position > Constants.ExFlipThreshold
                && !this.IsAtPosition(Turret1.getPosition(), Constants.TurretDefault, .05)) {
            MoveHorizontal(TurretHorizontal.Center);
            Wait(250);
        }

        currentTurretHeight = extakeFlip1Position;
        ExtakeFlip1.setPosition(extakeFlip1Position);
        ExtakeFlip2.setPosition(extakeFlip2Position);
        timer.reset();
        timer.startTime();
        this.LogTelemetry("Current Turret Position 1: ", extakeFlip1Position);
        this.LogTelemetry("Current Turret Position 2: ", extakeFlip2Position);
    }

    public void MoveVertical(TurretHeight height) {
        if(height == TurretHeight.Flipped)
            MoveVertical(Constants.ExtakeFlipOut, Constants.ExtakeFlipOut2);
        else if(height == TurretHeight.Low)
            MoveVertical(Constants.ExtakeFlipLow, Constants.ExtakeFlipLow2);
        else if(height == TurretHeight.Default)
            MoveVertical(Constants.ExtakeFlipIn, Constants.ExtakeFlipIn2);
        else if(height == TurretHeight.CycleVertical)
            MoveVertical(Constants.ExtakeFlipCycle, Constants.ExtakeFlipCycle2);
        else
            MoveVertical(Constants.ExtakeFlipCycle, Constants.ExtakeFlipCycle2);
    }

    public void MoveHorizontalOffset(double offset) {
        currentTurretHorizontal += offset;
        currentTurretHeight = CheckBoundries(currentTurretHeight);
        MoveHorizontal(currentTurretHorizontal);
    }

    public void MoveHorizontal(double position) {
        if(currentTurretHeight > Constants.ExFlipThreshold || CanMoveHorizontal()) {
            Turret1.setPosition(position);
            currentTurretHorizontal = position;
            LogTelemetry("Turret Horizontal: ", Turret1.getPosition());
        }
        else
            LogTelemetry("Can't move turret horizontal", null);
    }

    public boolean CanMoveHorizontal(){
        if(ExtakeFlip1.getPosition() >= Constants.ExFlipThreshold){
            return false;
        }
        else if(Turret1.getPosition() <= Constants.ExFlipThreshold && timer.milliseconds() < Constants.ExFlipTimerThreshold_Milliseconds){
            return false;
        }
        else
            return true;
    }

    public void MoveHorizontal(TurretHorizontal horizontal) {
        if(horizontal == TurretHorizontal.Left)
            MoveHorizontal(Constants.TurretLeft);
        else if(horizontal == TurretHorizontal.Right)
            MoveHorizontal(Constants.TurretRight);
        else if(horizontal == TurretHorizontal.Center)
            MoveHorizontal(Constants.TurretDefault);
        else if(horizontal == TurretHorizontal.CycleHorizontal)
            MoveHorizontal(Constants.TurretHorizontalCycle);
        else if(horizontal == TurretHorizontal.AutoRight)
            MoveHorizontal(Constants.AutoRight);

    }

    public void OpenClaw() {
        Claw.setPosition(Constants.ClawOpen);
        LogTelemetry("Claw Open: ", Claw.getPosition());
    }

    public void CloseClaw() {
        Claw.setPosition(Constants.ClawClosed);
        LogTelemetry("Claw Closed: ", Claw.getPosition());
    }

    public boolean IsClawOpen() {
        if(Claw.getPosition() == Constants.ClawOpen)
            return true;
        else
            return false;
    }

    public boolean IsClawClosed() {
        if(Claw.getPosition() == Constants.ClawClosed)
            return true;
        else
            return false;
    }

    private void SetSlidePosition(double slideIn, double slideIn2){
        SlideExtension.setPosition(slideIn);
        SlideExtension2.setPosition(slideIn2);
        LogTelemetry("Slide Position1: ", slideIn);
        LogTelemetry("Slide Position2: ", slideIn2);
    }

    public void SlideIn(){
        SetSlidePosition(Constants.SlideIn, Constants.SlideIn2);
    }

    public void SlideOut(){
        SetSlidePosition(Constants.SlideOut, Constants.SlideOut2);
    }

    public void SlideMid() { SetSlidePosition(Constants.SlideMid, Constants.SlideMid2); }

    public boolean IsSlideOut() {
        return this.IsAtPosition(Math.abs(Constants.SlideOut),
                Math.abs(SlideExtension.getPosition()),
                0.0);
    }

    public boolean IsAtVerticalPosition(double position, double buffer) {
        return this.IsAtPosition(Math.abs(position),
                Math.abs(ExtakeFlip1.getPosition()),
                buffer);
    }
    public boolean IsAtHorizontalPosition(double position, double buffer) {
        return this.IsAtPosition(Math.abs(position),
                Math.abs(Turret1.getPosition()),
                buffer);
    }

}
