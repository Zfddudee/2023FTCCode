package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    public enum liftPos {
        Bottom,
        Low,
        Middle,
        High,
        ManualStep
    }
     liftPos LiftPos = liftPos.Bottom;
public void LiftCases(HardwareMap hardwareMap) {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


    switch (LiftPos){
        case Bottom:
            drive.liftMotorL.setTargetPosition(Constants.LiftBottom);
            drive.liftMotorR.setTargetPosition(Constants.LiftBottom);
            drive.liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) drive.liftMotorL).setVelocity(Constants.LowVelocity);
            ((DcMotorEx) drive.liftMotorR).setVelocity(Constants.LowVelocity);
            break;
        case Low:
            drive.liftMotorL.setTargetPosition(Constants.LiftLow);
            drive.liftMotorR.setTargetPosition(Constants.LiftLow);
            drive.liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) drive.liftMotorL).setVelocity(Constants.HighVelocity);
            ((DcMotorEx) drive.liftMotorR).setVelocity(Constants.HighVelocity);
            break;
        case Middle:
            drive.liftMotorL.setTargetPosition(Constants.LiftMid);
            drive.liftMotorR.setTargetPosition(Constants.LiftMid);
            drive.liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) drive.liftMotorL).setVelocity(Constants.HighVelocity);
            ((DcMotorEx) drive.liftMotorR).setVelocity(Constants.HighVelocity);
            break;
        case High:
            drive.liftMotorL.setTargetPosition(Constants.LiftHigh);
            drive.liftMotorR.setTargetPosition(Constants.LiftHigh);
            drive.liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) drive.liftMotorL).setVelocity(Constants.HighVelocity);
            ((DcMotorEx) drive.liftMotorR).setVelocity(Constants.HighVelocity);
            break;
        case ManualStep:
            drive.liftMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.liftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.liftMotorL.setPower(-0.5);
            drive.liftMotorR.setPower(-0.5);
            break;

    }
}

}
