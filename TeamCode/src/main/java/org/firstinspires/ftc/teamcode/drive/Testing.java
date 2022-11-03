package org.firstinspires.ftc.teamcode.drive;

import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="Testing")
public class Testing extends LinearOpMode {


    // Some hardware access boilerplate; these would be initialized in init()
    // the lift motor, it's in RUN_TO_POSITION mode
    public DcMotor IntakeSlideMotor;
    public DcMotor IntakeFlipMotor;
    public DcMotor liftMotorR;
    public DcMotor liftMotorL;

    // the dump servo
    public CRServo IntakeWheels;
    public Servo IntakeFlip;
    public Servo Stomp;
    public Servo OdoRetractRight;
    public Servo OdoRetractLeft;
    public Servo OdoRetractRear;
    public Servo ExtakeFlip1;
    public Servo ExtakeFlip2;
    public Servo Turret1;
    public Servo Turret2;
    public Servo SlideExtension;
    public Servo Claw;

    public ColorRangeSensor IntakeSensor;

    double ClawPos;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IntakeFlipMotor = hardwareMap.get(DcMotor.class, "IntakeFlipMotor");
        IntakeSlideMotor = hardwareMap.get(DcMotor.class, "IntakeSlideMotor");
        liftMotorL = hardwareMap.get(DcMotor.class, "LiftMotorL");
        liftMotorR = hardwareMap.get(DcMotor.class, "LiftMotorR");

        IntakeWheels = hardwareMap.get(CRServo.class, "IntakeWheels");
        IntakeFlip = hardwareMap.get(Servo.class, "IntakeFlip");
        Stomp = hardwareMap.get(Servo.class, "Stomp");
        OdoRetractRight = hardwareMap.get(Servo.class, "OdoRetractRight");
        OdoRetractLeft = hardwareMap.get(Servo.class, "OdoRetractLeft");
        OdoRetractRear = hardwareMap.get(Servo.class, "OdoRetractRear");
        ExtakeFlip1 = hardwareMap.get(Servo.class, "ExtakeFlip1");
        ExtakeFlip2 = hardwareMap.get(Servo.class, "ExtakeFlip2");
        Turret1 = hardwareMap.get(Servo.class, "Turret1");
        Turret2 = hardwareMap.get(Servo.class, "Turret2");
        SlideExtension = hardwareMap.get(Servo.class, "SlideExtension");
        Claw = hardwareMap.get(Servo.class, "Claw");

        IntakeSensor = hardwareMap.get(ColorRangeSensor.class, "IntakeSensor");

        liftMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        Claw.scaleRange(0.3, 0.5);
//        SlideExtension.scaleRange(0, 0.5);
//        ExtakeFlip1.scaleRange(0, 0.7);
//        ExtakeFlip2.scaleRange(0, 0.7);
//        Turret1.scaleRange(0, 0.5);
//        Turret2.scaleRange(0, 0.5);


        IntakeSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeFlipMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeFlipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        waitForStart();

        while (!isStopRequested()) {
            if(gamepad1.options){
                IntakeFlipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                IntakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad1.a){

                IntakeSlideMotor.setTargetPosition(790);//-790
                IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) IntakeSlideMotor).setVelocity(2700);


//                IntakeFlip.setPosition(0);
            }
            if(gamepad1.x){
//                Stomp.setPosition(0.5);
//                Stomp.setPosition(1);
                IntakeFlip.setPosition(0.9);
                IntakeWheels.setPower(1);
                IntakeFlipMotor.setTargetPosition(550);
                //300 for above cone
                IntakeFlipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) IntakeFlipMotor).setVelocity(2700);
            }
            if (gamepad1.y){
//                ExtakeFlip1.setPosition(1);
//                ExtakeFlip2.setPosition(0);
                IntakeSlideMotor.setTargetPosition(0);
                IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) IntakeSlideMotor).setVelocity(2700);
//                ExtakeFlip1.setPosition(0);
//                ExtakeFlip2.setPosition(1);
            }
            if(gamepad1.b){
//                Turret1.setPosition(1);
//                Turret2.setPosition(0);
                IntakeFlip.setPosition(0.2);
                IntakeWheels.setPower(0.5);
                IntakeFlipMotor.setTargetPosition(0);
                IntakeFlipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) IntakeFlipMotor).setVelocity(600);
                IntakeSlideMotor.setTargetPosition(500);
                IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) IntakeSlideMotor).setVelocity(2700);

            }

            if (gamepad1.left_bumper){
                Claw.setPosition(0.4);
                SlideExtension.setPosition(500);
            }
            if(gamepad1.right_bumper){
//open                Claw.setPosition(0.5);
                        Claw.setPosition(0);
            }

//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x,
//                            -gamepad1.right_stick_x
//                    )
//            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("SensorValue", IntakeSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("IntakeMotorPos", IntakeSlideMotor.getCurrentPosition());
            telemetry.addData("LiftMotorR", liftMotorR.getCurrentPosition());
            telemetry.addData("LiftMotorL", liftMotorL.getCurrentPosition());
            telemetry.addData("IntakeFlipMotor", IntakeFlipMotor.getCurrentPosition());
            telemetry.addData("IntakeFlip", IntakeFlip.getPosition());
            telemetry.addData("Stomp", Stomp.getPosition());
            telemetry.addData("OdoRetractRight", OdoRetractRight.getPosition());
            telemetry.addData("OdoRetractLeft", OdoRetractLeft.getPosition());
            telemetry.addData("OdoRetractRear", OdoRetractRear.getPosition());
            telemetry.addData("ExtakeFlip1", ExtakeFlip1.getPosition());
            telemetry.addData("ExtakeFlip2", ExtakeFlip2.getPosition());
            telemetry.addData("Turret1", Turret1.getPosition());
            telemetry.addData("Turret2", Turret2.getPosition());
            telemetry.addData("SlideExtension", SlideExtension.getPosition());
            telemetry.addData("Claw", ClawPos);
            telemetry.update();



    }









        }
    }

