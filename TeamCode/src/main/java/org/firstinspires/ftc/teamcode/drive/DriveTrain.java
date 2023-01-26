package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

public class DriveTrain extends BaseRobot {

    //region Private Variables
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private VoltageSensor batteryVoltageSensor;
    private Servo Stomp, OdoRetractRight, OdoRetractLeft, OdoRetractRear;
    private BNO055IMU imu;
    //endregion

    //region Constants & Statics
    private final PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    private final PIDCoefficients HEADING_PID = new PIDCoefficients(5, 0, 0);
    private final double VX_WEIGHT = 1;
    private final double VY_WEIGHT = 1;
    private final double OMEGA_WEIGHT = 1;
    private final double lateralMultiplier = 1.0;
    //endregion

    public DriveTrain(HardwareMap map, Telemetry tel) {
        super(map, tel);
//        MapHardware();
    }

    @Override
    protected void MapHardware() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        Stomp = hardwareMap.get(Servo.class, "Stomp");
        OdoRetractRight = hardwareMap.get(Servo.class, "OdoRetractRight");
        OdoRetractLeft = hardwareMap.get(Servo.class, "OdoRetractLeft");
        OdoRetractRear = hardwareMap.get(Servo.class, "OdoRetractRear");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        //Reversing motor directions as needed
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    /**
     * Whenever there is a variable implemented into the code,
     * it determines the direction it travels, as well as the power per individual wheel motor
     * @param drivePower
     */
    public void Move(Pose2d drivePower){
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

           vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    //region Private Methods to control moving the robot
    private void setDrivePower( Pose2d drivePower) {
        List<Double> powers = MecanumKinematics.robotToWheelVelocities(
                drivePower,
                1.0,
                1.0,
                lateralMultiplier
        );
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }

    private void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    private void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    private void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }
    //endregion

    ///region Odometry Wheel Control
    public void OdoSideWheelsUp() {
        OdoRetractLeft.setPosition(Constants.OdoUp);
        OdoRetractRight.setPosition(Constants.OdoUp);
    }

    public void OdoSideWheelsDown() {
        OdoRetractLeft.setPosition(Constants.OdoDown);
        OdoRetractRight.setPosition(Constants.OdoDown);
    }

    public void OdoRearWheelUp() {
        OdoRetractRear.setPosition(Constants.OdoUp);
    }

    public void OdoRearWheelDown() {
        OdoRetractRear.setPosition(Constants.OdoDown);
    }

    public double GetRightOdoPosition() {
        return OdoRetractRight.getPosition();
    }

    public double GetLeftOdoPosition() {
        return OdoRetractLeft.getPosition();
    }

    public double GetRearOdoPosition() {
        return OdoRetractRear.getPosition();
    }
    ///endregion

    public double GetRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public double GetExternalHeadingVelocity() {
        // To work around an SDK bug, use -zRotationRate in place of xRotationRate
        // and -xRotationRate in place of zRotationRate (yRotationRate behaves as
        // expected). This bug does NOT affect orientation.
        //
        // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
        return (double) -imu.getAngularVelocity().xRotationRate;
    }

    public void StompDown() {
        Stomp.setPosition(Constants.StompDown);
    }

    public void StompUp() {
        Stomp.setPosition(Constants.StompUp);
    }
}