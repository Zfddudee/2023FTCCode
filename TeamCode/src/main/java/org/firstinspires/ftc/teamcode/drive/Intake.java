package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.CRServo;
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
    private CRServo IntakeWheels;
    private Servo IntakeFlip;
    private ColorRangeSensor IntakeSensor;

    public Intake(HardwareMap map, Telemetry tel) {
        super(map, tel);
    }

    @Override
    protected void MapHardware() {
        IntakeFlipMotor = hardwareMap.get(DcMotor.class, "IntakeFlipMotor");
        IntakeSlideMotor = hardwareMap.get(DcMotor.class, "IntakeSlideMotor");

        IntakeWheels = hardwareMap.get(CRServo.class, "IntakeWheels");
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
        this.LogTelemetry("Intake Sensor Distance(MM): ", GetSensorDistance(DistanceUnit.MM));
        return IntakeSensor.getDistance(unit);
    }

    public boolean SlowIntakeWheels () {
        if(GetSensorDistanceMM() <= Constants.IntakeWheelSensor) {
            IntakeWheels.setPower(Constants.IntakeWheelsSlow);
            return true;
        }
        else
            return false;
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
        this.LogTelemetry("Slide position: ", position);
    }
    ///endregion


    ///region Wheels Intake Flip
    /**
     * This is the servo that flips the intake wheels downwards
      */
    public void FlipDown() {
        IntakeFlip.setPosition(Constants.ServoIntakeFlipIntaking);
    }

    /**
     * This is the servo that flips the intake wheels upwards
     */
    public void FlipUp() {
        IntakeFlip.setPosition(Constants.ServoIntakeFlipExchanging);
    }
    ///endregion

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
        if(GetIntakePosition() <= (position - buffer) && GetIntakePosition() >= (position + buffer))
            return true;
        else
            return false;
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
        this.LogTelemetry("Intake Position: ", position);
    }
    ///endregion

    ///region Wheel Spin Direction
    /**
     *  This is the servo that spins the intake wheels inwards
     */
    public void IntakeSpinIn() {
        IntakeWheels.setPower(Constants.IntakeWheelsIn);
    }

    /**
     *  This is the servo that spins the intake wheels outwards
     */
    public void IntakeSpinOut() {
        IntakeWheels.setPower(Constants.IntakeWheelsOut);
    }

    /**
     * This stops the servo that spins the intake wheels
     */
    public void IntakeSpinStop() {
        IntakeWheels.setPower(Constants.IntakeWheelStop);
    }
    ///endregion
}
