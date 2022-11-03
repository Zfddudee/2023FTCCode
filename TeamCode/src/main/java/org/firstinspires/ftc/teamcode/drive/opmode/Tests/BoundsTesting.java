package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class BoundsTesting extends LinearOpMode {

    boolean toggle = false;
    boolean Reverse = false;
    public int ReverseMult = 1;

    public double BoundXLeft = -60;
    public double BoundXRight = 60;
    public double BoundYBottom = -60;
    public double BoundYTop = 60;
    public double XStick;
    public double YStick;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if(gamepad1.start){
            if(toggle == false){
                toggle = true;
                Reverse = !Reverse;
        }else{
            toggle = false;
        }
            if(Reverse){
            ReverseMult = -1;
            } else{
                ReverseMult = 1;
            }

        }

        while (!isStopRequested()) {
            XStick = gamepad1.left_stick_x;
            YStick = gamepad1.left_stick_y;
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -YStick * ReverseMult,
                            -XStick * ReverseMult,
                            -gamepad1.right_stick_x * ReverseMult
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

        }
        Pose2d position = drive.getPoseEstimate(); //Gets position of robot
        //Code to set the position of the code barriers
        if(gamepad1.dpad_up){
            BoundYTop = position.getY();
        }
        if(gamepad1.dpad_down){
            BoundYBottom = position.getY();
        }
        if(gamepad1.dpad_right){
            BoundXRight = position.getX();
        }
        if(gamepad1.dpad_left){
            BoundXLeft = position.getX();
        }

        //X positive barrier
        if(gamepad1.left_stick_x > 0 && position.getX() >= BoundXRight && position.getHeading() >= 0 && position.getHeading() <= 180){
            XStick = 0;
        }
        if(gamepad1.left_stick_y > 0 && position.getX() >= BoundXRight && position.getHeading() >= 90 && position.getHeading() <= 270){
            YStick = 0;
        }
        if(gamepad1.left_stick_x < 0 && position.getX() >= BoundXRight && position.getHeading() >= 180 && position.getHeading() <= 360){
            XStick = 0;
        }
        if(gamepad1.left_stick_y < 0 && position.getX() >= BoundXRight && position.getHeading() >= 270 && position.getHeading() <= 450){
            YStick = 0;
        }

        //X negative barrier
        if(gamepad1.left_stick_x < 0 && position.getX() <= BoundXLeft && position.getHeading() >= 0 && position.getHeading() <= 180){
            XStick = 0;
        }
        if(gamepad1.left_stick_y < 0 && position.getX() <= BoundXLeft && position.getHeading() >= 90 && position.getHeading() <= 270){
            YStick = 0;
        }
        if(gamepad1.left_stick_y > 0 && position.getX() <= BoundXLeft && position.getHeading() >= 180 && position.getHeading() <= 360){
            YStick = 0;
        }
        if(gamepad1.left_stick_y > 0 && position.getX() <= BoundXLeft && position.getHeading() >= 270 && position.getHeading() <= 450){
            YStick = 0;
        }

        //Y positive barrier
        if(gamepad1.left_stick_x > 0 && position.getY() >= BoundYTop && position.getHeading() >= 0 && position.getHeading() <= 180){
            XStick = 0;
        }
        if(gamepad1.left_stick_y > 0 && position.getY() >= BoundYTop && position.getHeading() >= 90 && position.getHeading() <= 270){
            YStick = 0;
        }
        if(gamepad1.left_stick_x < 0 && position.getY() >= BoundYTop && position.getHeading() >= 180 && position.getHeading() <= 360){
            XStick = 0;
        }
        if(gamepad1.left_stick_y < 0 && position.getY() >= BoundYTop && position.getHeading() >= 270 && position.getHeading() <= 450){
            YStick = 0;
        }

        //X negative barrier
        if(gamepad1.left_stick_x < 0 && position.getY() <= BoundYBottom && position.getHeading() >= 0 && position.getHeading() <= 180){
            XStick = 0;
        }
        if(gamepad1.left_stick_y < 0 && position.getY() <= BoundYBottom && position.getHeading() >= 90 && position.getHeading() <= 270){
            YStick = 0;
        }
        if(gamepad1.left_stick_y > 0 && position.getY() <= BoundYBottom && position.getHeading() >= 180 && position.getHeading() <= 360){
            YStick = 0;
        }
        if(gamepad1.left_stick_y > 0 && position.getY() <= BoundYBottom && position.getHeading() >= 270 && position.getHeading() <= 450){
            YStick = 0;
        }
    }
}
