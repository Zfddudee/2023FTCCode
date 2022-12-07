package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "TeleOpSubSystemTest", group ="Testing")

public class TeleOpSubSystemTest  extends LinearOpMode {

    private Turret turret;
    private Lift lift;
    private DriveTrain driveTrain;
    private Intake intake;

    public TeleOpSubSystemTest() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        driveTrain = new DriveTrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        waitForStart();
        ElapsedTime time = new ElapsedTime();
        time.startTime();


        while (!isStopRequested()) {
            if(gamepad1.a) {
                ///region Intake to Exchange Test
                turret.MoveVertical(Turret.TurretHeight.Low);
                turret.OpenClaw();
                sleep(5000);
                intake.SlideMotorOut();
                sleep(1000);
                intake.IntakeLow();
                sleep(5000);
                intake.FlipDown();
                intake.AutoCloseClaw();
                sleep(5000);
                intake.IntakeOut();
                sleep(5000);
                intake.IntakeLow();
                sleep(5000);
                intake.FlipUp();
                intake.IntakeIn();
                sleep(5000);
                intake.SlideMotorExchange();
                ///endregion
            }
            else if(gamepad1.b) {
                ///region Exchange to Extake Test
                turret.MoveVertical(Turret.TurretHeight.Default);
                turret.CloseClaw();
                intake.OpenClaw();
                lift.MoveLift(Lift.LiftHeight.Medium);
                lift.MoveLift(Lift.LiftHeight.High);
                ///endregion
            }
            else if(gamepad1.x) {
                ///region Extake
                turret.MoveVertical(Turret.TurretHeight.Flipped);
                sleep(5000);
                turret.SlideOut();
                sleep(2000);
                turret.MoveHorizontal(Turret.TurretHorizontal.Left);
                sleep(5000);
                turret.MoveHorizontal(Turret.TurretHorizontal.Right);
                sleep(5000);
                turret.MoveHorizontal(Turret.TurretHorizontal.Center);
                sleep(2000);
                turret.SlideIn();
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
                    -gamepad1.left_stick_y * -1,
                    -gamepad1.left_stick_x * -1,
                    -gamepad1.right_stick_x
            ));

            if(gamepad1.dpad_up || gamepad1.dpad_down)
                lift.MoveLift((gamepad1.dpad_up) ? 1 : -1);

            if(gamepad1.dpad_left || gamepad1.dpad_right)
                turret.MoveHorizontalOffset((gamepad1.dpad_left)?1 : -1);

            if(gamepad1.right_bumper || gamepad1.left_bumper) {
                if(gamepad1.right_bumper)
                    turret.SlideOut();
                else
                    turret.SlideIn();
            }

            //telemetry.addData("Current slide pos:", intake.GetCurrentSlidePosition());
            telemetry.update();
        }
    }
}
