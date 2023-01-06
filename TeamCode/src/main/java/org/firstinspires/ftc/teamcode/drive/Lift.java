package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//import org.apache.commons.math3.analysis.function.Constant;
//import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Lift extends BaseRobot{
    public enum LiftHeight{
        High,
        AutoHigh,
        Medium,
        Default,
        Low
    }

    private DcMotor liftMotorR, liftMotorL;

    public Lift(HardwareMap map, Telemetry tel){
        super(map, tel);
    }

    public void MoveLift(LiftHeight height) {
         if(height == LiftHeight.Default)
            MoveLift(Constants.LiftDefault);
        else if(height == LiftHeight.High)
            MoveLift(Constants.LiftHigh);
         else if(height == LiftHeight.AutoHigh)
             MoveLift(Constants.AutoLiftHigh);
        else if(height == LiftHeight.Low)
            MoveLift(Constants.LiftLow);
    }

    public void MoveLift(int position) {
        int velocity;


        if (position == 0){
            velocity = 1500;
            liftMotorR.setTargetPosition(position);
            liftMotorL.setTargetPosition(position);
            liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) liftMotorL).setVelocity(velocity);
            ((DcMotorEx) liftMotorR).setVelocity(velocity);
        }else{
            liftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Constants.liftError1 = position - liftMotorR.getCurrentPosition();
            Constants.liftError2 = position - liftMotorL.getCurrentPosition();
            Constants.liftPower1 = Constants.liftError1 * Constants.liftGainP;
            Constants.liftPower2 = Constants.liftError2 * Constants.liftGainP;
            liftMotorR.setPower(Constants.liftPower1);
            liftMotorL.setPower(Constants.liftPower2);

//            if (position == 0){
//                Constants.liftGainP = Constants.liftGainP0;
//            }else{
//                Constants.liftGainP = Constants.liftGainPUp;
//            }
        }



    }

    public void Telemetry(){
        this.LogTelemetry("Current Lift Left Position: ", liftMotorL.getCurrentPosition());
        this.LogTelemetry("Current Lift Right Position: ", liftMotorR.getCurrentPosition());
        this.LogTelemetry("LiftMotorR Amps", ((DcMotorEx) liftMotorR).getCurrent(CurrentUnit.AMPS));
        this.LogTelemetry("LiftMotorL Amps", ((DcMotorEx) liftMotorL).getCurrent(CurrentUnit.AMPS));
    }

    public void WaitTillCompleteMoveLift() {
        this.WaitTillComplete(liftMotorR, Constants.TimeOutTime);
        this.WaitTillComplete(liftMotorL, Constants.TimeOutTime);
    }

    @Override
    protected void MapHardware() {
        liftMotorL = hardwareMap.get(DcMotor.class, "LiftMotorL");
        liftMotorR = hardwareMap.get(DcMotor.class, "LiftMotorR");

        liftMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        liftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void Reset() {
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        liftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}