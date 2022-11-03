package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Disabled
@Autonomous(name = "RedCarouselTrajectory", preselectTeleOp = "19589_TeleOp")


public class RightRedSide extends LinearOpMode {



    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //start bot at pose x = -30, y = -64, heading 90 degrees
        Pose2d startPose = new Pose2d(35, -64, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence TrajectoryRW = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(60, -64, 90))
                .lineToSplineHeading(new Pose2d(60, 0, 180))
                .lineToSplineHeading(new Pose2d(60, -5, 150))
                .build();


        waitForStart();
        if(!isStopRequested())
            drive.followTrajectorySequence(TrajectoryRW);

    }
}
