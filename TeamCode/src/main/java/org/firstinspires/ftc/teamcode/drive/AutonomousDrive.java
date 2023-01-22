package org.firstinspires.ftc.teamcode.drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutonomousDrive {
    enum DriveSpeed{
        Fast,
        Conservative
    };
    enum DriveDirection {
        Left,
        Right
    }

    private DriveSpeed speed;
    private DriveDirection direction;
    private HardwareMap hardwareMap;

    //region Variables
    // These are the headings
    private double d1;
    private double d2;
    private  double d3;

    // This is an x value
    private double c1;
    // This is a y value
    private double c2;
    // This is a y value
    private double c3;
    // This is an x value
    private double c4;
    // This is a y value
    private double c5;
    //endregion

    public AutonomousDrive(HardwareMap map){

        hardwareMap = map;
    }

    public void Go(DriveSpeed newSpeed, DriveDirection newDirection) {
        direction = newDirection;
        speed = newSpeed;
        Go();
    }
    public void Go(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //start bot at pose x = 30, y = 64, heading 90 degrees
        Pose2d startPose = GetStartingPosition();

        drive.setPoseEstimate(startPose);
        TrajectorySequence TrajectoryOut;
        if(speed == DriveSpeed.Conservative)
            TrajectoryOut = SetConservativePath(drive, startPose);
        else
            TrajectoryOut = SetAggressivePath(drive, startPose);

        drive.followTrajectorySequence(TrajectoryOut);

    }

    private Pose2d GetStartingPosition() {
        if(direction == DriveDirection.Right)
            // This is the Right Starting Position
            return new Pose2d(35, 62, Math.toRadians(270));
        else
            // This is the Left Starting Position
            return new Pose2d(-35, -61, Math.toRadians(90));
    }

    private TrajectorySequence SetConservativePath(SampleMecanumDrive drive, Pose2d startingPosition){
        if(direction ==DriveDirection.Right) {
            d1 = 270;
            d2 = 180;
            d3 = 0;

            // This is an x value
            c1 = 35;
            // This is a y value
            c2 = 20;
            // This is a y value
            c3 = 13;
            // This is an x value
            c4 = 47;
            // This is a y value
            c5 = 12;
        }
        else {
            ///TODO: Enter in the left VARS
        }

        TrajectorySequence TrajectoryOut = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(c1, c2, Math.toRadians(d1)))
                .lineToSplineHeading(new Pose2d(c1, c3, Math.toRadians(d2)))
                .splineToConstantHeading(new Vector2d(c4, c5), Math.toRadians(d3))
                .build();
        return TrajectoryOut;
    }

    private TrajectorySequence SetAggressivePath(SampleMecanumDrive drive, Pose2d startingPosition){
        if(direction ==DriveDirection.Right) {
            d1 = 180;
            d2 = 0;

            // This is an x value
            c1 = 35;
            // This is a y value
            c2 = 20;
            // This is a x value
            c3 = 47;
            // This is an y value
            c4 = 12;

        }
        else {
            ///TODO: Enter in the left VARS
        }

        TrajectorySequence TrajectoryOut = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(c1, c2, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c3, c4), Math.toRadians(d2))
                .build();
        return TrajectoryOut;
    }

}
