package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

abstract class BaseRobot {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;

    public BaseRobot(HardwareMap map, Telemetry tel){
        hardwareMap = map;
        telemetry = tel;
    }
    public BaseRobot(HardwareMap map){
       this(map, null);
    }

    protected void LogTelemetry(String caption, Object value){
        if(telemetry != null)
            telemetry.addData(caption, value);
    }

    protected abstract void MapHardware();

}
