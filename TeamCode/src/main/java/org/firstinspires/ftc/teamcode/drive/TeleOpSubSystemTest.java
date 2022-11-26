package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "TeleOpSubSystemTest")

public class TeleOpSubSystemTest  extends LinearOpMode {

    private Turret turret;
    private Lift lift;
    private DriveTrain driveTrain;
    private Intake intake;

    public TeleOpSubSystemTest() {
        turret = new Turret(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        driveTrain = new DriveTrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        ElapsedTime time = new ElapsedTime();
        time.startTime();


        while (!isStopRequested()) {
            if(gamepad1.a) {
                ///region Intake to Exchange Test
                turret.MoveVertical(Turret.TurretHeight.Low);
                turret.OpenClaw();
                intake.SlideMotorOut();
                intake.IntakeLow();
                intake.FlipDown();
                intake.IntakeSpinIn();
                intake.IntakeOut();
                intake.IntakeLow();
                intake.IntakeSpinStop();
                intake.FlipUp();
                intake.IntakeIn();
                intake.SlideMotorExchange();
                ///endregion
            }
            else if(gamepad1.b) {
                ///region Exchange to Extake Test
                turret.MoveVertical(Turret.TurretHeight.Default);
                turret.CloseClaw();
                intake.IntakeSpinOut();
                lift.MoveLift(Lift.LiftHeight.Medium);
                intake.IntakeSpinStop();
                lift.MoveLift(Lift.LiftHeight.High);
                ///endregion
            }
            else if(gamepad1.x) {
                ///region Extake
                turret.MoveVertical(Turret.TurretHeight.Flipped);
                turret.MoveHorizontal(Turret.TurretHorizontal.Left);
                turret.MoveHorizontal(Turret.TurretHorizontal.Right);
                turret.MoveHorizontal(Turret.TurretHorizontal.Center);
                ///endregion
            }
            else if(gamepad1.y) {
                ///region Return
                turret.MoveVertical(Turret.TurretHeight.Low);
                lift.MoveLift(Lift.LiftHeight.Medium);
                turret.OpenClaw();
                lift.MoveLift(Lift.LiftHeight.Default);
                ///endregion
            }

            driveTrain.Move(new Pose2d(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
               -gamepad1.right_stick_x
            ));
//
//            if(gamepad2.dpad_up || gamepad2.dpad_down)
//                lift.MoveLift((gamepad2.dpad_up) ? 1 : -1);
//
//            if(gamepad2.dpad_left || gamepad2.dpad_right)
//                intake.
        }
    }
}
