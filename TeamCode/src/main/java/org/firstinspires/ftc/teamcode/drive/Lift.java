package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends BaseRobot{
    public enum LiftDirection{
        Up,
        Down
    }

    private DcMotor liftMotorR, liftMotorL;

    public Lift(HardwareMap map, Telemetry tel){
        super(map, tel);
        MapHardware();
    }

    public void MoveLift(LiftDirection direction){
        //TODO: implement - this will call the other MoveLift method but will only move it to the complete bottom or top
    }
    public void MoveLift(LiftDirection direction, double distance){
        //TODO: implement
    }

    @Override
    protected void MapHardware() {
        liftMotorL = hardwareMap.get(DcMotor.class, "LiftMotorL");
        liftMotorR = hardwareMap.get(DcMotor.class, "LiftMotorR");

        liftMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
