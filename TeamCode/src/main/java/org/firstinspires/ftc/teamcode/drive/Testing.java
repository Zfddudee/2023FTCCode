package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.hardware.usb.ArmableUsbDevice;

@Config
@TeleOp(name="Testing", group="Testing")
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
    public Servo SlideExtension;
    public Servo SlideExtension2;
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
        SlideExtension = hardwareMap.get(Servo.class, "SlideExtension");
        SlideExtension2 = hardwareMap.get(Servo.class, "SlideExtension2");
        Claw = hardwareMap.get(Servo.class, "Claw");

//        IntakeSensor = hardwareMap.get(ColorRangeSensor.class, "IntakeSensor");

        // extake flip servo 1 is normal toque 2 is 5 turn

        liftMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
//        Claw.scaleRange(0, 0.5);
//        SlideExtension.scaleRange(0, 0.5);
//        ExtakeFlip1.scaleRange(0, 0.7);
//        ExtakeFlip2.scaleRange(0, 0.7);
//        Turret1.scaleRange(0, 0.5);
//        Turret2.scaleRange(0, 0.5);


        IntakeSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeFlipMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeFlipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double turretPose = 0.5;

        int liftTarget = 0;




        waitForStart();

        while (!isStopRequested()) {
            drive.Turret1.setPosition(turretPose);
            drive.ExtakeFlip1.setPosition(Constants.ArmTarget1);
            drive.ExtakeFlip2.setPosition(Constants.ArmTarget2);

            Constants.liftError1 = liftTarget - liftMotorR.getCurrentPosition();
            Constants.liftError2 = liftTarget - liftMotorL.getCurrentPosition();
            Constants.liftPower1 = Constants.liftError1 * Constants.liftGainP;
            Constants.liftPower2 = Constants.liftError2 * Constants.liftGainP;
            liftMotorR.setPower(Constants.liftPower1);
            liftMotorL.setPower(Constants.liftPower2);

            if (liftTarget == 0){
                Constants.liftGainP = Constants.liftGainP0;
            }else{
                Constants.liftGainP = Constants.liftGainPUp;
            }

            Constants.ArmError1 = Constants.SetArmTarget1 - Constants.ArmTarget1;
            Constants.ArmError2 = Constants.SetArmTarget2 - Constants.ArmTarget2;

            if (Constants.ArmError1 >= 0.01){
                Constants.ArmTarget1 = Constants.ArmTarget1 + Constants.ArmStepOver;
            } else if (Constants.ArmError1 <= -0.01){
                Constants.ArmTarget1 = Constants.ArmTarget1 - Constants.ArmStepOver;
            }

            if (Constants.ArmError2 >= 0.01){
                Constants.ArmTarget2 = Constants.ArmTarget2 + Constants.ArmStepOver;
            } else if (Constants.ArmError2 <= -0.01){
                Constants.ArmTarget2 = Constants.ArmTarget2 - Constants.ArmStepOver;
            }



            if(gamepad1.options){
                IntakeFlipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                IntakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad1.a){

                liftTarget = Constants.LiftHigh;
//    lift            liftMotorR.setTargetPosition(Constants.LiftHigh);//-790
//                liftMotorL.setTargetPosition(Constants.LiftHigh);//-790
//                liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ((DcMotorEx) liftMotorL).setVelocity(Constants.HighVelocity);
//                ((DcMotorEx) liftMotorR).setVelocity(Constants.HighVelocity);

//                ExtakeFlip2.setPosition(1);



//                IntakeFlip.setPosition(0);
            }
            if(gamepad1.x){
                liftTarget = Constants.LiftMid;
//                Stomp.setPosition(0.5);
//                Stomp.setPosition(1);
//                IntakeFlip.setPosition(0.9);
//                IntakeWheels.setPower(1);
//                IntakeFlipMotor.setTargetPosition(550);
//                //300 for above cone
//                IntakeFlipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ((DcMotorEx) IntakeFlipMotor).setVelocity(2700);
//                Turret2.setPosition(1);
//      lift          liftMotorR.setTargetPosition(Constants.LiftMid);//-790
//                liftMotorL.setTargetPosition(Constants.LiftMid);//-790
//                liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ((DcMotorEx) liftMotorL).setVelocity(Constants.HighVelocity);
//                ((DcMotorEx) liftMotorR).setVelocity(Constants.HighVelocity);
            }
            if (gamepad1.y){
                liftTarget = Constants.LiftDefault;
//                ExtakeFlip1.setPosition(1);
//                ExtakeFlip2.setPosition(0);
//                IntakeSlideMotor.setTargetPosition(0);
//                IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ((DcMotorEx) IntakeSlideMotor).setVelocity(2700);
//                ExtakeFlip1.setPosition(0);
//                ExtakeFlip2.setPosition(1);
//                Turret1.setPosition(1);
//   lift             liftMotorR.setTargetPosition(Constants.LiftDefault);//-790
//                liftMotorL.setTargetPosition(Constants.LiftDefault);//-790
//                liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ((DcMotorEx) liftMotorL).setVelocity(Constants.LowVelocity);
//                ((DcMotorEx) liftMotorR).setVelocity(Constants.LowVelocity);
            }

            if(gamepad1.b){
                drive.IntakeSlideMotor.setTargetPosition(Constants.IntakeOut);//-790
                drive.IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) drive.IntakeSlideMotor).setVelocity(Constants.HighVelocity);
                drive.IntakeFlip.setPosition(Constants.ServoIntakeFlipIntaking);
                drive.Claw.setPosition(Constants.ClawOpen);
            }
            if(gamepad1.right_stick_y > 0.5){
                drive.IntakeFlipMotor.setTargetPosition(Constants.IntakeFlipsLow);
                drive.IntakeFlipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) drive.IntakeFlipMotor).setVelocity(Constants.HighVelocity);
            }
            if(gamepad1.right_stick_x > 0.5){
                drive.IntakeFlipMotor.setTargetPosition(Constants.IntakeFlips);
                drive.IntakeFlipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) drive.IntakeFlipMotor).setVelocity(Constants.HighVelocity);
            }
            if(gamepad1.right_stick_y < -0.5){
                drive.IntakeFlipMotor.setTargetPosition(Constants.IntakeFlipsIn);
                drive.IntakeFlipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) drive.IntakeFlipMotor).setVelocity(Constants.HighVelocity);
            }
            if(gamepad1.right_stick_x < -0.5){
//                drive.IntakeSlideMotor.setTargetPosition(Constants.IntakeIn);//-790
//                drive.IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ((DcMotorEx) drive.IntakeSlideMotor).setVelocity(Constants.HighVelocity);
                drive.IntakeFlip.setPosition(Constants.ServoIntakeFlipExchanging);
                drive.Claw.setPosition(Constants.ClawOpen);
            }

            if (gamepad1.left_bumper){
                Claw.setPosition(Constants.ClawClosed);
            }
            if(gamepad1.right_bumper){
                Claw.setPosition(Constants.ClawOpen);
            }
            if (gamepad1.left_trigger > 0.5){
                Constants.SetArmTarget1 = Constants.ExtakeFlipIn;
                Constants.SetArmTarget2 = Constants.ExtakeFlipIn2;
            }
            if (gamepad1.right_trigger > 0.5){
                Constants.SetArmTarget1 = Constants.ExtakeFlipOut;
                Constants.SetArmTarget2 = Constants.ExtakeFlipOut2;
            }
            if(gamepad1.dpad_right){
                turretPose = Constants.TurretRight;
            }
            if(gamepad1.dpad_left){
                turretPose = Constants.TurretLeft;
            }
            if(gamepad1.dpad_up){
                drive.SlideExtension.setPosition(Constants.SlideOut);
                drive.SlideExtension2.setPosition(Constants.SlideOut2);
            }
            if(gamepad1.dpad_down){
                drive.SlideExtension.setPosition(Constants.SlideIn);
                drive.SlideExtension2.setPosition(Constants.SlideIn2);
            }
            if(gamepad1.share){
                drive.SlideExtension.setPosition(Constants.SlideIn);
                drive.SlideExtension2.setPosition(Constants.SlideIn2);
                liftMotorR.setTargetPosition(Constants.LiftDefault);//-790
                liftMotorL.setTargetPosition(Constants.LiftDefault);//-790
                liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) liftMotorL).setVelocity(Constants.LowVelocity);
                ((DcMotorEx) liftMotorR).setVelocity(Constants.LowVelocity);
                turretPose = Constants.TurretDefault;
            }
            if(gamepad1.left_stick_x > 0.5){
                turretPose = turretPose + Constants.TurretStepOver;
            }
            if(gamepad1.left_stick_x < -0.5){
                turretPose = turretPose - Constants.TurretStepOver;
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("ArmPos1", Constants.ArmTarget1);
            telemetry.addData("ArmPos2", Constants.ArmTarget2);
//            telemetry.addData("SensorValue", IntakeSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("IntakeMotorPos", IntakeSlideMotor.getCurrentPosition());
            telemetry.addData("IntakeSlide Amps", ((DcMotorEx) drive.IntakeSlideMotor).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("LiftMotorR", liftMotorR.getCurrentPosition());
            telemetry.addData("LiftMotorL", liftMotorL.getCurrentPosition());
            telemetry.addData("Target Position", liftTarget);
            telemetry.addData("Power 1", Constants.liftPower1);
            telemetry.addData("Power 2", Constants.liftPower2);
            telemetry.addData("LiftMotorR Amps", ((DcMotorEx) drive.liftMotorR).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("LiftMotorL Amps", ((DcMotorEx) drive.liftMotorL).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("IntakeFlipMotor", IntakeFlipMotor.getCurrentPosition());
            telemetry.addData("IntakeFlip Amps", ((DcMotorEx) drive.IntakeFlipMotor).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("IntakeFlip", IntakeFlip.getPosition());
            telemetry.addData("Stomp", Stomp.getPosition());
            telemetry.addData("OdoRetractRight", OdoRetractRight.getPosition());
            telemetry.addData("OdoRetractLeft", OdoRetractLeft.getPosition());
            telemetry.addData("OdoRetractRear", OdoRetractRear.getPosition());
            telemetry.addData("ExtakeFlip1", ExtakeFlip1.getPosition());
            telemetry.addData("ExtakeFlip2", ExtakeFlip2.getPosition());
            telemetry.addData("Turret1", Turret1.getPosition());
            telemetry.addData("SlideExtension", SlideExtension.getPosition());
            telemetry.addData("Claw", ClawPos);
            telemetry.addData("TouchpadX", gamepad1.touchpad_finger_1_x);
            telemetry.addData("TouchpadY", gamepad1.touchpad_finger_1_y);
            telemetry.addData("leftstickx", gamepad1.left_stick_x);
            telemetry.update();



    }









        }
    }

