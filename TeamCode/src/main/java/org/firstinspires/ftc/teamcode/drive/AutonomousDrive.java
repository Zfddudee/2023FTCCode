package org.firstinspires.ftc.teamcode.drive;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Config
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
    private SampleMecanumDrive drive;

    //region Variables
    // These are the headings
    public double d1;
    public double d2;
    public  double d3;

    // This is an x value
    public double c1;
    // This is a y value
    public double c2;
    // This is a y value
    public double c3;
    // This is an x value
    public double c4;
    // This is a y value
    public double c5;
    //
    public double c6;
    //
    public double c7;
    //
    public double c8;
    //
    public double c9;
    //
    public double c10;
    //
    public double c11;
    //endregion

    private Pose2d endPosition;

    public AutonomousDrive(HardwareMap map){
        hardwareMap = map;
    }

    public void Go(DriveSpeed newSpeed, DriveDirection newDirection) {
        direction = newDirection;
        speed = newSpeed;
        Go();
    }
    public void Go(){
        drive = new SampleMecanumDrive(hardwareMap);
        SetVariables();

        //start bot at pose x = 30, y = 64, heading 90 degrees
        Pose2d startPose = GetStartingPosition();

        drive.setPoseEstimate(startPose);
        TrajectorySequence TrajectoryOut;
        if(speed == DriveSpeed.Conservative)
            TrajectoryOut = SetConservativePath(drive, startPose);
        else
            TrajectoryOut = SetAggressivePath(drive, startPose);

        drive.followTrajectorySequence(TrajectoryOut);
        endPosition = TrajectoryOut.end();
    }

    private Pose2d GetStartingPosition() {
        if(direction == DriveDirection.Right)
            // This is the Right Starting Position
            return new Pose2d(-35, 62, Math.toRadians(90));
        else
            // This is the Left Starting Position
            return new Pose2d(-35, -61, Math.toRadians(90));
    }

    private void SetVariables(){
        if(direction ==DriveDirection.Right) {
            // Right Coordinates
            d1 = 90;
            d2 = 180;
            d3 = 270;

            // This is an x value
            c1 = -35;
            // This is a y value
            c2 = 20;
            // This is a y value
            c3 = 13;
            // This is an x value
            c4 = -47;
            // This is a y value
            c5 = 12;
            //
            c6 = -57;
            //
            c7 = -59;
            //
            c8 = 23;
            //
            c9 = -38;
            //
            c10 = -18;
            //
            c11 = -12;
        }
        else{
            // Left Coordinates
            d1 = 90;
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
    }

    private TrajectorySequence SetConservativePath( SampleMecanumDrive drive, Pose2d startingPosition){
         TrajectorySequence TrajectoryOut = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(c1, c2, Math.toRadians(d1)))
                .lineToSplineHeading(new Pose2d(c1, c3, Math.toRadians(d2)))
                .splineToConstantHeading(new Vector2d(c4, c5), Math.toRadians(d3))
                .build();
        return TrajectoryOut;
    }

    private TrajectorySequence SetAggressivePath(SampleMecanumDrive drive, Pose2d startingPosition){
        TrajectorySequence TrajectoryOut = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(c1, c2, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c3, c4), Math.toRadians(d2))
                .build();
        return TrajectoryOut;
    }

    public void Park(int Position){
        SetVariables();
        if(endPosition == null)
            endPosition = new Pose2d(0.0);
        TrajectorySequence sequence;
        if(Position == 1)
            sequence = SetStationOne(endPosition);
        else if(Position == 2)
            sequence = SetStationTwo(endPosition);
        else
            sequence = SetStationThree(endPosition);

        drive.followTrajectorySequence(sequence);
    }

    private TrajectorySequence SetStationOne(Pose2d startPose){
        TrajectorySequence TrajectoryX = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(c6, c5, Math.toRadians(d3)))
                .splineToConstantHeading(new Vector2d(c7, c8), Math.toRadians(d1))
                .build();
        return TrajectoryX;
    }

    private TrajectorySequence SetStationTwo(Pose2d startPose){
        TrajectorySequence TrajectoryY = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(c9, c5, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c1, c8), Math.toRadians(d1))
                .build();
        return TrajectoryY;
    }
    private TrajectorySequence SetStationThree(Pose2d startPose){
        TrajectorySequence TrajectoryZ = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(c10, c5, Math.toRadians(d2)))
                .lineToSplineHeading(new Pose2d(c11, c5, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c11, c8), Math.toRadians(d1))
                .build();
        return TrajectoryZ;
    }

}
