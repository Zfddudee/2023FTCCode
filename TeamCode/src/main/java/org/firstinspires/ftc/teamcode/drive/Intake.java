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

    enum ClawState
    {
        Open,
        Close
    }

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
//        ((DcMotorEx)IntakeFlipMotor).setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,);
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


    public void SlideMotorAutoOut(int SlidePose) {
        SetSlidePosition(Constants.IntakeOutAuto1);
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
    public void SlideMotorWall() {
        SetSlidePosition(Constants.IntakeWall);
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
    public boolean IsIntakeSlideAtPosition(int position, int buffer) {
        return this.IsAtPosition(position, GetCurrentSlidePosition(), buffer);
    }
    public void SetSlidePositionOffset(int offset) {
        SetSlidePosition(GetCurrentSlidePosition() + offset);
    }

    private void SetSlidePosition(int position) {
        IntakeSlideMotor.setTargetPosition(position);
        IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) IntakeSlideMotor).setVelocity(Constants.HighVelocity);

    }
    public void SlideStop(){
        ((DcMotorEx) IntakeSlideMotor).setVelocity(0);
    }
    private void SetFlip(double position) {
        IntakeFlip.setPosition(position);
    }
    ///endregion

    ///region Wheels Intake Flip

    public void ToggleFlip() {
        if(IntakeFlip.getPosition() == Constants.ServoIntakeFlipIntaking)
            FlipUp();
        else
            FlipDown();
    }

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
    public void AutoFlipUp() {
        SetFlip(Constants.AutoServoIntakeFlipExchanging);
    }
    ///endregion

    public void WaitTillSlideIsComplete() {
        this.WaitTillComplete(IntakeSlideMotor, Constants.TimeOutTime);
    }

    ///region Entire Intake Flip
    /**
     * This is the motor that flips the entire intake outwards
     */
    public void IntakeOut() {
        SetIntakeFlipPosition(Constants.IntakeFlips, Constants.HighVelocity);
    }
    public void AutoIntakeOut(int Position){
        SetIntakeFlipPosition(Position, Constants.HighVelocity);
    }

    /**
     * This is the motor that flips the intake into a position that prepares to grab cones
     */
    public void IntakeLow() {
        SetIntakeFlipPosition(Constants.IntakeFlipsLow, Constants.HighVelocity);
    }

    public void IntakeOut1() {
        SetIntakeFlipPosition(Constants.IntakeFlips1, Constants.HighVelocity);
    }
    /**
     * This is the motor that flips the entire intake inwards to its default position
     */
    public void IntakeIn() {
        SetIntakeFlipPosition(Constants.IntakeIn, Constants.LowVelocity);
    }

    public int GetIntakeFlipPosition() {
        return IntakeFlipMotor.getCurrentPosition();
    }

    public boolean IsIntakeFlipAtPosition(int position, int buffer) {
        return this.IsAtPosition(position, GetIntakeFlipPosition(), buffer);
    }

    public void SetIntakeFlipPosition(int positionOffset) {
        int newPosition = IntakeFlipMotor.getCurrentPosition() + positionOffset;
        int velocity = Constants.LowVelocity;
        if(positionOffset > 0)
            velocity = Constants.HighVelocity;
        SetIntakeFlipPosition(newPosition, velocity);
    }

    public void SetIntakeFlipPosition(int position, int velocity) {
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
        //todo find and add in color to if statement.
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
        if (IntakeFlipMotor.getCurrentPosition() > Constants.IntakeFlipsLow - 200) {
            IntakeClaw.setPosition(Constants.IntakeClawOpen);
        }
        else
            IntakeClaw.setPosition(Constants.IntakeClawPartial);

    }

    public void OpenCloseClaw() {
        if(IntakeClaw.getPosition() == Constants.IntakeClawOpen)
            CloseClaw();
        else
            OpenClaw();
    }

    /**
     * Function to flip intake in
     */
    public void IntakeNewExchange() {
        SetIntakeFlipPosition(IntakeNewExchange, Constants.HighVelocity);
    }

}
