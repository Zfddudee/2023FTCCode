package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.Constants.IntakeNewExchange;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends BaseRobot {

    private DcMotor IntakeSlideMotor, IntakeFlipMotor;
    private Servo IntakeClaw;
    private Servo IntakeFlip;
    private ColorRangeSensor IntakeSensor;

    public Intake(HardwareMap map, Telemetry tel) {
        super(map, tel);
    }

    @Override
    protected void MapHardware() {
        IntakeFlipMotor = hardwareMap.get(DcMotor.class, "IntakeFlipMotor");
        IntakeSlideMotor = hardwareMap.get(DcMotor.class, "IntakeSlideMotor");

        IntakeClaw = hardwareMap.get(Servo.class, "IntakeClaw");
        IntakeFlip = hardwareMap.get(Servo.class, "IntakeFlip");

        IntakeSensor = hardwareMap.get(ColorRangeSensor.class, "IntakeSensor");

        IntakeSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeFlipMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //setting zero power behaviors
        IntakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeFlipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeFlipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double GetSensorDistanceMM() {
        return GetSensorDistance(DistanceUnit.MM);
    }


    public double GetSensorDistance(DistanceUnit unit) {

        return IntakeSensor.getDistance(unit);
    }
    public double GetSensorColor() {

        return IntakeSensor.getLightDetected();
    }

    public void Telemetry(){
        this.LogTelemetry("Intake Sensor Distance(MM): ", GetSensorDistance(DistanceUnit.MM));
        this.LogTelemetry("Intake Sensor Color: ", GetSensorColor());
        this.LogTelemetry("Slide position: ", IntakeSlideMotor.getCurrentPosition());
        this.LogTelemetry("Flip Position: ", IntakeFlipMotor.getCurrentPosition());
        this.LogTelemetry("IntakeClawPosition", IntakeClaw.getPosition());
    }

    ///region Slide Motor
    /**
     * This controls the motor to move the slide outwards
     */
    public void SlideMotorOut() {
        SetSlidePosition(Constants.IntakeOut);
    }

    /**
     * This controls the motor to move the slide inwards to its default
     */
    public void SlideMotorIn() {
        SetSlidePosition(Constants.IntakeIn);
    }

    /**
     * This controls the motor to move the slide in a position to exchange
     */
    public void SlideMotorExchange() {
        SetSlidePosition(Constants.IntakeExchanging);
    }

    public boolean IsSlideMotorBusy() {
        return IntakeSlideMotor.isBusy();
    }

    /**
     * Gets the current position of the slide motor
     * @return
     */
    public int GetCurrentSlidePosition() {
        return IntakeSlideMotor.getCurrentPosition();
    }

    public void SetSlidePositionOffset(int offset) {
        SetSlidePosition(GetCurrentSlidePosition() + offset);
    }

    private void SetSlidePosition(int position) {
        IntakeSlideMotor.setTargetPosition(position);
        IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) IntakeSlideMotor).setVelocity(Constants.HighVelocity);

    }
    private void SetFlip(double position) {
        IntakeFlip.setPosition(position);
    }
    ///endregion


    ///region Wheels Intake Flip
    /**
     * This is the servo that flips the intake wheels downwards
      */
    public void FlipDown() {
        SetFlip(Constants.ServoIntakeFlipIntaking);
    }

    /**
     * This is the servo that flips the intake wheels upwards
     */
    public void FlipUp() {
        SetFlip(Constants.ServoIntakeFlipExchanging);
    }
    ///endregion

    public void WaitTillSlideIsComplete() {
        this.WaitTillComplete(IntakeSlideMotor, Constants.TimeOutTime);
    }

    ///region Entire Intake Flip
    /**
     * This is the motor that brings the entire intake outwards
     */
    public void IntakeOut() {
        SetIntakePosition(Constants.IntakeFlips, Constants.HighVelocity);
    }

    /**
     * This is the motor that brings the intake into a position that prepares to grab cones
     */
    public void IntakeLow() {
        SetIntakePosition(Constants.IntakeFlipsLow, Constants.HighVelocity);
    }

    public void IntakeOut1() {
        SetIntakePosition(Constants.IntakeFlips1, Constants.HighVelocity);
    }
    /**
     * This is the motor that brings the entire intake inwards to its default position
     */
    public void IntakeIn() {
        SetIntakePosition(Constants.IntakeIn, Constants.LowVelocity);
    }

    public int GetIntakePosition() {
        return IntakeFlipMotor.getCurrentPosition();
    }

    public boolean IsIntakeAtPosition(int position, int buffer) {
        return this.IsAtPosition(position, GetIntakePosition(), buffer);
    }

    public void SetIntakePosition(int positionOffset) {
        int newPosition = IntakeFlipMotor.getCurrentPosition() + positionOffset;
        int velocity = Constants.LowVelocity;
        if(positionOffset > 0)
            velocity = Constants.HighVelocity;
        SetIntakePosition(newPosition, velocity);
    }

    private void SetIntakePosition(int position, int velocity) {
        IntakeFlipMotor.setTargetPosition(position);
        IntakeFlipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) IntakeFlipMotor).setVelocity(velocity);
    }

    public void WaitTillIntakeMotorIsComplete() {
        this.WaitTillComplete(IntakeFlipMotor, Constants.TimeOutTime);
    }

    ///endregion

    ///region Wheel Spin Direction
    /**
     *  This is the servo that closes the intake claw
     */
    public boolean AutoCloseClaw() {
        //todo find and add in color  to if statement.
        if(GetSensorDistanceMM() <= Constants.IntakeWheelSensor){
            CloseClaw();
            return true;
        }
        else
            return false;
    }

    public void CloseClaw(){
        IntakeClaw.setPosition(Constants.IntakeClawClosed);
    }
    /**
     *  This is the servo that opens the intake claw
     */
    public void OpenClaw() {
        if (IntakeFlipMotor.getCurrentPosition() > Constants.IntakeFlipsLow) {
            IntakeClaw.setPosition(Constants.IntakeClawOpen);
        }
        else
            IntakeClaw.setPosition(Constants.IntakeClawOpen * Constants.IntakeClawPartial);

    }

    public void OpenCloseClaw() {
        if(IntakeClaw.getPosition() == Constants.IntakeClawOpen)
            CloseClaw();
        else
            OpenClaw();
    }

    public void IntakeNewExchange() {
        SetIntakePosition(IntakeNewExchange, Constants.LowVelocity);
    }

}
