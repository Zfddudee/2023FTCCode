package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Turret extends BaseRobot{
    public enum TurretHeight{
        Default,
        Low,
        Flipped
    }

    public enum TurretHorizontal{
        Left,
        Right,
        Center
    }

    private double currentTurretHeight;
    private double currentTurretHorizontal;

    private Servo ExtakeFlip1, ExtakeFlip2, Turret1, Claw;

    public Turret(HardwareMap map, Telemetry tel) {
        super(map, tel);
//        MapHardware();
        currentTurretHeight = 0.0;
        currentTurretHorizontal = Constants.TurretDefault;
    }

    @Override
    protected void MapHardware() {
        ExtakeFlip1 = hardwareMap.get(Servo.class, "ExtakeFlip1");
        ExtakeFlip2 = hardwareMap.get(Servo.class, "ExtakeFlip2");
        Turret1 = hardwareMap.get(Servo.class, "Turret1");
        Claw = hardwareMap.get(Servo.class, "Claw");
        //setting servo scale ranges
        Claw.scaleRange(Constants.ClawLowRange, Constants.ClawHighRange);
    }

    private double CheckBoundries(double position) {
        if(position > 1)
            position = 1.0;
        if(position < 0)
            position = 0.0;
        return position;
    }

    public void MoveVerticalOffset(double offset) {
        currentTurretHeight += offset;
        CheckBoundries(currentTurretHeight);
        MoveVertical(currentTurretHeight);
    }

    public void MoveVertical(double position) {
        ExtakeFlip2.setPosition(position);
        currentTurretHeight = position;
    }

    public void MoveVertical(TurretHeight height) {
        if(height == TurretHeight.Flipped)
            MoveVertical(Constants.ExtakeFlipOut);
        else if(height == TurretHeight.Low)
            MoveVertical(Constants.ExtakeFlipLow);
        else
            MoveVertical(Constants.ExtakeFlipIn);
    }

    public void MoveHorizontalOffset(double offset) {
        currentTurretHorizontal += offset;
        currentTurretHeight = CheckBoundries(currentTurretHeight);
        MoveVertical(currentTurretHorizontal);
    }

    public void MoveHorizontal(double position) {
        Turret1.setPosition(position);
        currentTurretHorizontal = position;
    }

    public void MoveHorizontal(TurretHorizontal horizontal) {
        if(horizontal == TurretHorizontal.Left)
            MoveHorizontal(Constants.TurretLeft);
        else if(horizontal == TurretHorizontal.Right)
            MoveHorizontal(Constants.TurretRight);
        else
            MoveHorizontal(Constants.TurretDefault);
    }

    public void OpenClaw() {
        Claw.setPosition(Constants.ClawOpen);
    }

    public void CloseClaw() {
        Claw.setPosition(Constants.ClawClosed);
    }
}
