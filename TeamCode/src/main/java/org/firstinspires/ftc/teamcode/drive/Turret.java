package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret extends BaseRobot{
    private Servo ExtakeFlip1, ExtakeFlip2, Turret1, Claw;

    public Turret(HardwareMap map, Telemetry tel) {
        super(map, tel);
        MapHardware();
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

    public void MoveVertical(double position) {
        ExtakeFlip2.setPosition(Constants.ExtakeFlipOut);
    }
}
