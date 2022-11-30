package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

abstract class BaseRobot {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;

    public BaseRobot(HardwareMap map, Telemetry tel){
        hardwareMap = map;
        telemetry = tel;
        MapHardware();
    }
    public BaseRobot(HardwareMap map){
       this(map, null);
    }

    protected void LogTelemetry(String caption, Object value){
        if(telemetry != null) {
            telemetry.addData(caption, value);
            //telemetry.update();
        }
    }

    protected boolean IsAtPosition(double targetPosition, double currentPosition, double buffer) {
      if(currentPosition >= (targetPosition - buffer) && currentPosition <= (targetPosition + buffer))
            return true;
        else
            return false;
    }

    protected abstract void MapHardware();

    protected void WaitTillComplete(DcMotor motor, int timeoutMM) {
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while(!motor.isBusy() || timer.milliseconds() < timeoutMM) {
        }
    }
}
