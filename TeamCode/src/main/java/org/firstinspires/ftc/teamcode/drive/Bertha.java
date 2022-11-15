package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bertha{
    enum Direction{
        Forward,
        Backward,
        Up,
        Down
    }
    enum ClawDirection{
        Open,
        Close
    }

    //region Robot Hardware
    public DcMotor IntakeSlideMotor, IntakeFlipMotor;
    public CRServo IntakeWheels;
    public Servo IntakeFlip, Stomp, OdoRetractRight, OdoRetractLeft, OdoRetractRear,  SlideExtension;
    public ColorRangeSensor IntakeSensor;
    private BNO055IMU imu;
    //endregion

    ///region Robot objects
    private Lift lift;
    private DriveTrain driveTrain;
    //TODO: Create objects for Intake, Stomp, DriveTrain(Wheels), Turret(includes claw)
    //endregion

    public Bertha(HardwareMap map, Telemetry tel){
        lift = new Lift(map, tel);
        driveTrain = new DriveTrain(map, tel);

    }

    public void Drive(Pose2d drivePower) {
       driveTrain.Move(drivePower);
    }

    public void ExtendFlipMotor(){
        //TODO: Implement code to extend flip motor
    }
    public void RetractFlipMotor(){
        //TODO: Implement retract flip motor
    }
    public void StartIntakeWheels(Direction direction){
        //TODO: Implement code to move intake wheels
    }
    public void StopIntakeWheels(){
        //TODO: Stop intake wheels
    }
    public void SetClawDirection(ClawDirection direction){
        //TODO: set the claw direction
    }
    public void MoveLift(Lift.LiftHeight height) throws Exception {
        lift.MoveLift(height);
    }
    public void MoveLift(int offSet) throws Exception {
        lift.MoveLift(offSet);
    }

    /**
     * Maps all the hardware to private objects
     */
    protected void MapHardware(){
        /*
        IntakeFlipMotor = hardwareMap.get(DcMotor.class, "IntakeFlipMotor");
        IntakeSlideMotor = hardwareMap.get(DcMotor.class, "IntakeSlideMotor");

        IntakeWheels = hardwareMap.get(CRServo.class, "IntakeWheels");
        IntakeFlip = hardwareMap.get(Servo.class, "IntakeFlip");
        Stomp = hardwareMap.get(Servo.class, "Stomp");
        OdoRetractRight = hardwareMap.get(Servo.class, "OdoRetractRight");
        OdoRetractLeft = hardwareMap.get(Servo.class, "OdoRetractLeft");
        OdoRetractRear = hardwareMap.get(Servo.class, "OdoRetractRear");
        SlideExtension = hardwareMap.get(Servo.class, "SlideExtension");

        IntakeSensor = hardwareMap.get(ColorRangeSensor.class, "IntakeSensor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: reverse any motors using DcMotor.setDirection()

        IntakeSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeFlipMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //setting zero power behaviors
        IntakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeFlipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */
    }


}
