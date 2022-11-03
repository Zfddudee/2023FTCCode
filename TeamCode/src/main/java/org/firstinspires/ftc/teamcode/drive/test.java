package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Disabled
@Autonomous(name = "Red", preselectTeleOp = "19589_TeleOp")


public class test extends LinearOpMode {



    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //start bot at pose x = -30, y = -64, heading 90 degrees
        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(15,25,Math.toRadians(40)))
                .lineToSplineHeading(new Pose2d(40,25,Math.toRadians(40)))
                .lineToSplineHeading(new Pose2d(15,45,Math.toRadians(90)))
                .build();

        waitForStart();
        if(!isStopRequested())
            drive.followTrajectorySequence(trajSeq);


    }
}
