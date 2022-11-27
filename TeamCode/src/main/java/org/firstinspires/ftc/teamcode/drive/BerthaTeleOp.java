package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BertaTeleOp")
public class BerthaTeleOp extends LinearOpMode {

    private Bertha bertha;

    @Override
    public void runOpMode() throws InterruptedException {
        bertha = new Bertha(hardwareMap, telemetry);

        waitForStart();
        while (!isStopRequested()) {
            ExecuteEachLoop();
            bertha.Execute();
            telemetry.update();
        }
    }

    private void ExecuteEachLoop()
    {
        //This moves the robot forwards, backwards, and strafes left and right
        //Left Joystick
        bertha.Move(new Pose2d(
                -gamepad1.left_stick_y * -1,
                -gamepad1.left_stick_x * -1,
                -gamepad1.right_stick_x
        ));

        if(gamepad1.x)
            bertha.WheelsSpinOut();
        else if(gamepad1.a)
            bertha.PreConePickUp();

    }
}
