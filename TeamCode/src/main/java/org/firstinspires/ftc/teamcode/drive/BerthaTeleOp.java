package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BerthaTeleOp", group="Bertha")
public class BerthaTeleOp extends LinearOpMode {

    private Bertha bertha;
    private boolean rightTrigger = false;
    private boolean touchpad = false;
    private boolean ButtonA = false;
    private boolean ButtonY = false;
    private boolean ButtonX = false;
    private boolean ButtonB = false;
    private boolean Share = false;
    private boolean Gamepad2X = false;

    @Override
    public void runOpMode() throws InterruptedException {
        bertha = new Bertha(hardwareMap, telemetry);
        waitForStart();
        while (!isStopRequested()) {
            try {
                GamePad1Loop();
                GamePad2Loop();
                bertha.RunOpMode();
            }
            catch (Exception ex)
            {
                telemetry.addData("ERROR in main loop", ex.getMessage());
                telemetry.addData("Inner exception", ex.getStackTrace());
            }
            telemetry.update();
        }
    }

    private void GamePad1Loop() {
        //This moves the robot forwards, backwards, and strafes left and right
        //Left Joystick
        bertha.Move(new Pose2d(
                -gamepad1.left_stick_y * -1 * Constants.DrivePower,
                -gamepad1.left_stick_x * -1 * Constants.DrivePower,
                -gamepad1.right_stick_x * Constants.DrivePower
        ));

        if(gamepad1.x && !ButtonX) {
            bertha.IntakeOpenClaw();
            ButtonX = true;
        }
        //opens intake claw
        else if(gamepad1.a && !ButtonA) {
                bertha.PreConePickUp();
                ButtonA = true;
        }
        //sets to going to pickup cone positions
//        else if(gamepad1.b)
        else if(gamepad1.y && !ButtonY) {
            bertha.PickUpOverRide();
            ButtonY = true;
        }
        //Brings arm back to overide if goes wrong
        else if(gamepad1.dpad_up)
            bertha.MoveIntake(-20);
        //manual move intake arm out
        else if(gamepad1.dpad_down)
            bertha.MoveIntake(20);
        //manual move intake arm in
        else if(gamepad1.dpad_right)
            bertha.MoveSlide(30);
        //manual move intake slide out
        else if(gamepad1.dpad_left)
            bertha.MoveSlide(-30);
        //manual move intake slide in
        else if(gamepad1.share && !Share) {
            bertha.Reset();
            Share = true;
        }
        //resets robot to moving state
        else if(gamepad1.right_bumper)
            bertha.StompDown();
        //puts stomp down
        else if(gamepad1.left_bumper)
            bertha.StompUp();
        //puts stomp up
        else if(gamepad1.touchpad && !touchpad) {
                bertha.MoveToExchange2();
                touchpad = true;
        }
        //started exchange proccess
//        else if(gamepad1.right_stick_button)

//            bertha.AutoExtake();

        //open/close claw toggle button
        else if(gamepad1.right_trigger != 0 && !rightTrigger) {
            if(!rightTrigger) {
                bertha.OpenCloseIntakeClaw();
                rightTrigger = true;
            }
        }
        else if(gamepad1.right_trigger == 0)
            rightTrigger = false;
        //flips intake wrist flip
        else if(gamepad1.right_stick_button)
            bertha.ToggleClawFlip();

        if (!gamepad1.touchpad)
            touchpad = false;
        if (!gamepad1.a)
            ButtonA = false;
        if (!gamepad1.b)
            ButtonB = false;
        if (!gamepad1.x)
            ButtonX = false;
        if (!gamepad1.y)
            ButtonY = false;
        if (!gamepad1.share)
            Share = false;
    }

    private void GamePad2Loop() {
        if(gamepad2.a)
            bertha.TeleOpCycle();
        else if(gamepad2.b)
            bertha.IntakeReturn();
        else if(gamepad2.y)
            bertha.LiftMedium();
        //opens extake claw
        else if(gamepad2.right_trigger > 0)
            bertha.OpenClaw();
        //closes extake claw
        else if(gamepad2.left_trigger > 0)
            bertha.CloseClaw();
        //moves turret to right position
        else if(gamepad2.right_bumper)
            bertha.TurretRight();
        //moves turret to left position
        else if(gamepad2.left_bumper)
            bertha.TurretLeft();
//        else if(gamepad2.right_stick_y != 0)
//            bertha.MoveLiftOffset(gamepad2.right_stick_y);
        //resets robot to moving position
        else if(gamepad2.share)
            bertha.Reset();
        //manually move arm up
        else if(gamepad2.dpad_up)
            bertha.TurretVertical(-Constants.TurretVerticalStepOver);
        //manually move arm down
        else if(gamepad2.dpad_down)
            bertha.TurretVertical(Constants.TurretVerticalStepOver);
        //manually move turret right
        else if(gamepad2.dpad_right)
            bertha.TurretHorizontal(Constants.TurretStepOver);
        //manually move turret left
        else if(gamepad2.dpad_left)
            bertha.TurretHorizontal(-Constants.TurretStepOver);
        //centers turret
        else if(gamepad2.touchpad)
            bertha.TurretCenter();
        else if(gamepad2.left_stick_button)
            bertha.ExchangeToExtake();
        //toggle between extake slide positions
        else if(gamepad2.x) {
            if (!Gamepad2X) {
                bertha.ExtakeSlideInOut();
                Gamepad2X = true;
            }
        }
        else if(!gamepad2.x)
            Gamepad2X = false;

    }
}
