package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Config
@TeleOp(name = "TeleOpMain", group="Testing")
public class TeleopMain2023 extends LinearOpMode {

    // An Enum is used to represent lift states.
    // (This is one thing enums are designed to do)
    public enum CycleState {
        START,
        IntakeFullDrop,
        IntakeIn,
        ExtakeTransfer,
        LiftUp,
        Drop,
        Retract,
        FullRetract
    }

    public enum StompState {
        Down,
        Up
    }

    public enum OdoState {
        Down,
        Up
    }

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    CycleState cycleState = CycleState.START;
    StompState stompState = StompState.Up;
    OdoState odoState = OdoState.Down;

    // Some hardware access boilerplate; these would be initialized in init()





    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.IntakeFlipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.IntakeFlipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.IntakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ElapsedTime timer = new ElapsedTime();
        timer.startTime();


        waitForStart();


        while (!isStopRequested()) {

            if (gamepad1.options) {
                drive.IntakeFlipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.IntakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad1.share) {
                drive.liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (drive.IntakeFlipMotor.getCurrentPosition() <= 50) {

            }
            if (drive.IntakeFlipMotor.getCurrentPosition() >= 500) {

            }

            switch (cycleState) {
                //Worked on
                case START:
                    // Waiting for a press
                    if (gamepad1.a) {
                        // x is pressed, start cycle opperation
                        drive.IntakeSlideMotor.setTargetPosition(Constants.IntakeOut);//-790
                        drive.IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ((DcMotorEx) drive.IntakeSlideMotor).setVelocity(Constants.HighVelocity);
                        drive.IntakeFlip.setPosition(Constants.ServoIntakeFlipIntaking);
                        drive.Claw.setPosition(Constants.ClawOpen);

                    }
                    if (Math.abs(drive.IntakeSlideMotor.getCurrentPosition() - Constants.IntakeOut) <= 100) {
                        drive.IntakeFlipMotor.setTargetPosition(Constants.IntakeFlipsLow);
                        drive.IntakeFlipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ((DcMotorEx) drive.IntakeFlipMotor).setVelocity(Constants.HighVelocity);
                        cycleState = CycleState.IntakeFullDrop;
                    }
                    break;

                //Worked on
                case IntakeFullDrop:
                    if (gamepad1.b) {
                        drive.IntakeFlipMotor.setTargetPosition(Constants.IntakeFlips);
                        //300 for above cone
                        drive.IntakeFlipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ((DcMotorEx) drive.IntakeFlipMotor).setVelocity(Constants.HighVelocity);
                        cycleState = CycleState.IntakeIn;
                    }
                    break;

                //Worked on
                case IntakeIn:
                    if (drive.IntakeSensor.getDistance(DistanceUnit.MM) <= 20) {
                        drive.IntakeFlip.setPosition(Constants.ServoIntakeFlipExchanging);

                        timer.reset();
                        timer.startTime();
                        cycleState = CycleState.ExtakeTransfer;
                    }
                    break;

                //Worked on
                case ExtakeTransfer:
                    drive.IntakeFlipMotor.setTargetPosition(0);
                    ((DcMotorEx) drive.IntakeFlipMotor).setVelocity(Constants.LowVelocity);
                    drive.IntakeFlipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (Math.abs(drive.IntakeFlipMotor.getCurrentPosition()) <= 20) {
                        drive.IntakeSlideMotor.setTargetPosition(Constants.IntakeExchanging);
                        if (timer.time() >= 0.15) {
                            drive.Claw.setPosition(Constants.ClawClosed);

                            if (timer.time() > 2) {
                                timer.reset();
                                timer.startTime();
                                cycleState = CycleState.LiftUp;
                            }
                        }
                    }
                    break;

                //Worked on
                case LiftUp:
                    drive.ExtakeFlip2.setPosition(Constants.ExtakeFlipOut);
                    drive.SlideExtension.setPosition(Constants.SlideOut);
                    drive.IntakeSlideMotor.setTargetPosition(Constants.IntakeOut);
                    if (!Constants.Left) {
                        drive.liftMotorR.setTargetPosition(Constants.LiftHigh);//-790
                        drive.liftMotorL.setTargetPosition(Constants.LiftHigh);//-790
                        drive.liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        drive.liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ((DcMotorEx) drive.liftMotorL).setVelocity(Constants.HighVelocity);
                        ((DcMotorEx) drive.liftMotorR).setVelocity(Constants.HighVelocity);


                        if (timer.seconds() > 1) {
                            drive.Turret1.setPosition(Constants.TurretRight);
                            timer.reset();
                            timer.startTime();
                            cycleState = CycleState.Drop;
                        }
                    } else {
                        drive.liftMotorR.setTargetPosition(Constants.LiftMid);//-790
                        drive.liftMotorL.setTargetPosition(Constants.LiftMid);//-790
                        drive.liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        drive.liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        if (timer.seconds() > 1) {
                            drive.Turret1.setPosition(Constants.TurretLeft);
                            timer.reset();
                            timer.startTime();
                            cycleState = CycleState.Drop;
                        }
                    }
                    break;

                //Worked on
                case Drop:
                    if (gamepad1.a) {
                        drive.Claw.setPosition(Constants.ClawOpen);
                        sleep(150);
                        //Retract here as well
                        drive.Turret1.setPosition(Constants.TurretDefault);
                        drive.SlideExtension.setPosition(Constants.SlideIn);

                        //TODO change name from bottom to default
                        drive.liftMotorR.setTargetPosition(Constants.LiftDefault);//-790
                        drive.liftMotorL.setTargetPosition(Constants.LiftDefault);//-790

                        drive.liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        drive.liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ((DcMotorEx) drive.liftMotorL).setVelocity(Constants.LowVelocity);
                        ((DcMotorEx) drive.liftMotorR).setVelocity(Constants.LowVelocity);
                        if (timer.time() >= 1) {
                            drive.ExtakeFlip2.setPosition(Constants.ExtakeFlipIn);
                            cycleState = CycleState.START;
                            timer.reset();
                            timer.startTime();
                        }
                    }
                    break;

                //
                case FullRetract:
                    drive.Turret1.setPosition(Constants.TurretDefault);

                    drive.Stomp.setPosition(Constants.StompUp);
                    drive.IntakeFlip.setPosition(Constants.ServoIntakeFlipExchanging);
                    drive.IntakeSlideMotor.setTargetPosition(Constants.IntakeIn);
                    drive.IntakeFlipMotor.setTargetPosition(Constants.IntakeFlipsIn);
                    drive.SlideExtension.setPosition(Constants.SlideIn);
                   
                   //TODO change name from bottom to default
                   drive.liftMotorL.setTargetPosition(Constants.LiftDefault);
                    drive.liftMotorR.setTargetPosition(Constants.LiftDefault);

                    drive.Claw.setPosition(Constants.ClawOpen);
                    drive.ExtakeFlip2.setPosition(Constants.ExtakeFlipIn);
                    cycleState = CycleState.START;
                    break;

                //
                default:
                    // should never be reached, as liftState should never be null
                    cycleState = CycleState.START;
            }

            if (Constants.readyToStomp && gamepad1.left_trigger >= 0.4) {
                switch (stompState) {

                    case Up:
                        drive.Stomp.setPosition(Constants.StompDown);
                        stompState = StompState.Down;
                        break;

                    case Down:
                        drive.Stomp.setPosition(Constants.StompUp);
                        stompState = StompState.Up;
                        break;

                    default:
                        // should never be reached, as liftState should never be null however there incase something fails and it reaches defualt
                        stompState = StompState.Up;
                }
                Constants.readyToStomp = false;
            } else if (gamepad1.left_trigger < 0.5) {
                Constants.readyToStomp = true;
            }

            if (Constants.odoReady && gamepad1.share) {
                switch (odoState) {
                    case Up:
                        drive.OdoRetractRear.setPosition(Constants.OdoDown);
                        drive.OdoRetractLeft.setPosition(Constants.OdoUp);
                        drive.OdoRetractRight.setPosition(Constants.OdoUp);
                        odoState = OdoState.Down;
                        break;

                    case Down:
                        drive.OdoRetractRear.setPosition(Constants.OdoUp);
                        drive.OdoRetractLeft.setPosition(Constants.OdoDown);
                        drive.OdoRetractRight.setPosition(Constants.OdoDown);
                        odoState = OdoState.Up;
                        break;

                    default:
                        // should never be reached, as liftState should never be null however there incase something fails and it reaches defualt
                        odoState = OdoState.Up;
                }
                Constants.odoReady = false;
            } else if (!gamepad1.start) {
                Constants.odoReady = true;
            }


            if (gamepad1.right_bumper && cycleState != CycleState.START) {
                cycleState = CycleState.START;
            }
            if (gamepad1.left_bumper) {
                cycleState = CycleState.FullRetract;
                stompState = StompState.Up;
            }
            //TODO: Lift motor zero, Intake motor zero, reverse intake when needed, Manual overides?,
            // Oranize code more, Possibly make separate classes to reference from to oranize more,
            // Use camera for allignment and possibly for intake?, Make possible to manually test individual motors if needed
        }
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * -1,
                        -gamepad1.left_stick_x * -1,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("Current states", cycleState);
        telemetry.addData("Stomp states", stompState);
        telemetry.addData("SensorValue", drive.IntakeSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("IntakeMotorPos", drive.IntakeSlideMotor.getCurrentPosition());
        telemetry.addData("LiftMotorR", drive.liftMotorR.getCurrentPosition());
        telemetry.addData("LiftMotorL", drive.liftMotorL.getCurrentPosition());
        telemetry.addData("+", drive.IntakeFlipMotor.getCurrentPosition());
        telemetry.addData("IntakeFlip", drive.IntakeFlip.getPosition());
        telemetry.addData("Stomp", drive.Stomp.getPosition());
        telemetry.addData("OdoRetractRight", drive.OdoRetractRight.getPosition());
        telemetry.addData("OdoRetractLeft", drive.OdoRetractLeft.getPosition());
        telemetry.addData("OdoRetractRear", drive.OdoRetractRear.getPosition());
        telemetry.addData("ExtakeFlip1", drive.ExtakeFlip1.getPosition());
        telemetry.addData("ExtakeFlip2", drive.ExtakeFlip2.getPosition());
        telemetry.addData("Turret1", drive.Turret1.getPosition());
        telemetry.addData("SlideExtension", drive.SlideExtension.getPosition());
        telemetry.addData("Claw", drive.Claw.getPosition());
        telemetry.update();


    }
}
