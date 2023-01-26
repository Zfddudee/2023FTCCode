package org.firstinspires.ftc.teamcode.drive;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

    //Sets right or left, or conservative or fast
    public void Go(DriveSpeed newSpeed, DriveDirection newDirection) {
        direction = newDirection;
        speed = newSpeed;
        Go();
    }

    //Setting up drive. and sets variables, starting position, initialized trajectory out, sets to aggressive or conservative path then follows trajectory.
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

    //Setting starting positions for right or left
    private Pose2d GetStartingPosition() {
        if(direction == DriveDirection.Right)
            // This is the Right Starting Position
            return new Pose2d(-35, 62, Math.toRadians(90));
        else
            // This is the Left Starting Position
            return new Pose2d(35, 62, Math.toRadians(90));
    }

    //Sets all position variables for sides.
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
            c4 = -46.5;
            // This is a y value
            c5 = 13;
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
            d2 = 0;
            d3 = 270;

            // This is an x value
            c1 = 35;
            // This is a y value
            c2 = 20;
            // This is a y value
            c3 = 13;
            // This is an x value
            c4 = 45.5;
            // This is a y value
            c5 = 11;
            //
            c6 = 57;
            //
            c7 = 59;
            //
            c8 = 23;
            //
            c9 = 38;
            //
            c10 = 18;
            //
            c11 = 12;
        }
    }

    //Attemp at correcting heading
//    public void TurnCorrect(){
//        TrajectorySequence Correct = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(c4, c5, Math.toRadians(d3)))
//                        .build();
//        drive.followTrajectorySequence(Correct);
//    }

    //Conservative path
    private TrajectorySequence SetConservativePath( SampleMecanumDrive drive, Pose2d startingPosition){
         TrajectorySequence TrajectoryOut = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(c1, c2, Math.toRadians(d1)))
                .lineToSplineHeading(new Pose2d(c1, c3, Math.toRadians(d2)))
                .splineToConstantHeading(new Vector2d(c4, c5), Math.toRadians(d3))
                .build();
        return TrajectoryOut;
    }

    //Aggressive path
    private TrajectorySequence SetAggressivePath(SampleMecanumDrive drive, Pose2d startingPosition){
        TrajectorySequence TrajectoryOut = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(c1, c2, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c4, c3), Math.toRadians(d2))
                .build();
        return TrajectoryOut;
    }

    //Setting position that park goes to based off camera and follows sequence
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

    //Position one path
    private TrajectorySequence SetStationThree(Pose2d startPose){
        TrajectorySequence TrajectoryX = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(c6, c5, Math.toRadians(d3)))
                .splineToConstantHeading(new Vector2d(c7, c8), Math.toRadians(d1))
                .build();
        return TrajectoryX;
    }

    //Position two path
    private TrajectorySequence SetStationTwo(Pose2d startPose){
        TrajectorySequence TrajectoryY = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(c9, c5, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c1, c8), Math.toRadians(d1))
                .build();
        return TrajectoryY;
    }

    //Position three path
    private TrajectorySequence SetStationOne(Pose2d startPose){
        TrajectorySequence TrajectoryZ = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(c10, c5, Math.toRadians(d2)))
                .lineToSplineHeading(new Pose2d(c11, c5, Math.toRadians(d1)))
                .splineToConstantHeading(new Vector2d(c11, c8), Math.toRadians(d1))
                .build();
        return TrajectoryZ;
    }

}
