package com.example.falconroboticstester;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class FalconRoboticsTester {


    /*

        int d1 = 270;

        int d2 = -180;

        int d3 = 0;

        int d4 = 0;

        // These values should be x = Positive, y = Positive

        // This is an x value
        int c1 = -12;
        // This is a y-value
        int c2 = -2;
        // This is an x-value
        int c3 = -16;
        // This is a y-value
        int c4 = 12;
        // This is an x-value
        int c5 = -25;
        // This is a y-value
        int c6 = 4;
        // This is a y-value
        int c7 = 0;
        // This is an x-value
        int c8 = -35;
        // This is a y-value
        int c9 = 20;
        // This is an x-value
        int c10 = -56;
        // This is an x-value
        int c11 = -54;
        // This is an x-value
        int c12 = -50;
        // This is an x-value
        int c13 = -46;
        // This is an x-value
        int c14 = -42;
        // This is a y-value
        int c15 = 36;
        // This is an x-value
        int c16 = -36;
        // This is a y-value
        int c17 = 32;
        // This is an x-value
        int c18 = -60;
    */
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-35, 60, Math.toRadians(270)))

                                        // The segment goes from the origin to the first extake
                                        .lineToSplineHeading(new Pose2d(-35, 20, Math.toRadians(270)))
                                        .splineToConstantHeading(new Vector2d(-25, 12), Math.toRadians(0))
                                        .lineToSplineHeading(new Pose2d(-16, 12, Math.toRadians(270)))
                                        .splineToConstantHeading(new Vector2d(-12, 4), Math.toRadians(270))
                                        .lineToSplineHeading(new Pose2d(-12, 0, Math.toRadians(270)))
                                        .waitSeconds(1)

                                        // This segment goes from extake #1 to intake #1
                                        .lineToSplineHeading(new Pose2d(-12, 2, Math.toRadians(270)))
                                        .splineToConstantHeading(new Vector2d(-16, 12), Math.toRadians(-180))
                                        .lineToSplineHeading(new Pose2d(-56, 12, Math.toRadians(0)))
                                        .waitSeconds(1)

                                        // This segment goes from intake #1 to extake #2
                                        .lineToSplineHeading(new Pose2d(-25, 12, Math.toRadians(270)))
                                        .lineToSplineHeading(new Pose2d(-16, 12, Math.toRadians(270)))
                                        .splineToConstantHeading(new Vector2d(-12, 4), Math.toRadians(270))
                                        .lineToSplineHeading(new Pose2d(-12, 0, Math.toRadians(270)))
                                        .waitSeconds(1)

                                        // This segment goes from extake #2 to intake #2
                                        .lineToSplineHeading(new Pose2d(-12, 2, Math.toRadians(270)))
                                        .splineToConstantHeading(new Vector2d(-16, 12), Math.toRadians(-180))
                                        .lineToSplineHeading(new Pose2d(-54, 12, Math.toRadians(0)))
                                        .waitSeconds(1)

                                        // This segment goes from intake #2 to extake #3
                                        .lineToSplineHeading(new Pose2d(-25, 12, Math.toRadians(270)))
                                        .lineToSplineHeading(new Pose2d(-16, 12, Math.toRadians(270)))
                                        .splineToConstantHeading(new Vector2d(-12, 4), Math.toRadians(270))
                                        .lineToSplineHeading(new Pose2d(-12, 0, Math.toRadians(270)))
                                        .waitSeconds(1)

                                        // This segment goes from extake #3 to intake #3
                                        .lineToSplineHeading(new Pose2d(-12, 2, Math.toRadians(270)))
                                        .splineToConstantHeading(new Vector2d(-16, 12), Math.toRadians(-180))
                                        .lineToSplineHeading(new Pose2d(-50, 12, Math.toRadians(0)))
                                        .waitSeconds(1)

                                        // This segment goes from intake #3 to extake #4
                                        .lineToSplineHeading(new Pose2d(-25, 12, Math.toRadians(270)))
                                        .lineToSplineHeading(new Pose2d(-16, 12, Math.toRadians(270)))
                                        .splineToConstantHeading(new Vector2d(-12, 4), Math.toRadians(270))
                                        .lineToSplineHeading(new Pose2d(-12, 0, Math.toRadians(270)))
                                        .waitSeconds(1)

                                        // This segment goes from extake #4 to intake #4
                                        .lineToSplineHeading(new Pose2d(-12, 2, Math.toRadians(270)))
                                        .splineToConstantHeading(new Vector2d(-16, 12), Math.toRadians(-180))
                                        .lineToSplineHeading(new Pose2d(-46, 12, Math.toRadians(0)))
                                        .waitSeconds(1)

                                        // This segment goes from intake #4 to extake #5
                                        .lineToSplineHeading(new Pose2d(-25, 12, Math.toRadians(270)))
                                        .lineToSplineHeading(new Pose2d(-16, 12, Math.toRadians(270)))
                                        .splineToConstantHeading(new Vector2d(-12, 4), Math.toRadians(270))
                                        .lineToSplineHeading(new Pose2d(-12, 0, Math.toRadians(270)))
                                        .waitSeconds(1)

                                        // This segment goes from extake #5 to intake #5
                                        .lineToSplineHeading(new Pose2d(-12, 2, Math.toRadians(270)))
                                        .splineToConstantHeading(new Vector2d(-16, 12), Math.toRadians(-180))
                                        .lineToSplineHeading(new Pose2d(-42, 12, Math.toRadians(0)))
                                        .waitSeconds(1)

                                        // This segment goes from intake #5 to extake #6
                                        .lineToSplineHeading(new Pose2d(-25, 12, Math.toRadians(270)))
                                        .lineToSplineHeading(new Pose2d(-16, 12, Math.toRadians(270)))
                                        .splineToConstantHeading(new Vector2d(-12, 4), Math.toRadians(270))
                                        .lineToSplineHeading(new Pose2d(-12, 0, Math.toRadians(270)))
                                        .waitSeconds(1)

/*
                                This code is for the parking at the end of the auto

                                // This segment parks the robot in spot #1
                                .lineToSplineHeading(new Pose2d(-12, 36, Math.toRadians(270)))

                                // This segment parks the robot in spot #2
                                .lineToSplineHeading(new Pose2d(-12, 32, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(-16, 36), Math.toRadians(-0))
                                .lineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(270)))
*/
                                        // This segment parks the robot in spot #3
                                        .lineToSplineHeading(new Pose2d(-12, 32, Math.toRadians(270)))
                                        .splineToConstantHeading(new Vector2d(-16, 36), Math.toRadians(-180))
                                        .lineToSplineHeading(new Pose2d(-60, 36, Math.toRadians(270)))


                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(180.95f)
                .addEntity(myBot)
                .start();

    }
}
