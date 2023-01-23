package com.example.falconroboticstester;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class FalconRoboticsTester {


    public static Vars GetVars(Mode mode){
        Vars v = new Vars();

        if(mode == Mode.Left1) {
            // LEFT RED
            v.Init(-35, -62, 0, -12, 0, 270);

            v.d1 = 90;
            v.d2 = 180;

            // This is an x value
            v.c1 = -35;
            // This is a y value
            v.c2 = -30;
            // This is an x value
            v.c3 = -45;
            // This is a y value
            v.c4 = -11.5;
            // This is an x value
            v.c5 = -52;

        }
        else if(mode == Mode.Right1) {
            // RIGHT RED
            //start bot at pose x = 30, y = 64, heading 90 degrees
            //start bot at pose x = 12, y = 0, heading 90 degrees
            v.Init(35, 62, 270, -12, 0, 90);

            v.d1 = 270;
            v.d2 = 0;

            // This is an x value
            v.c1 = 35;
            // This is a y value
            v.c2 = 20;
            // This is an x value
            v.c3 = 45;
            // This is a y value
            v.c4 = 11.5;
            // This is an x value
            v.c5 = 52;

        }
        else if(mode == Mode.SpeedTestRight) {
            v.Init(35, 62, 270, -12, 0, 90);

            v.d1 = 180;
            v.d2 = 0;

            v.c1 = 35;
            v.c2 = 20;
            v.c3 = 47;
            v.c4 = 12;

        }
        else if(mode == Mode.SpeedTestLeft) {}
        else if(mode == Mode.MiddleTestRight) {
            v.Init(-35, 62, 90, -12, 0, 90);

            v.d1 = 90;
            v.d2 = 180;
            v.d3 = 270;

            v.c1 = -35;
            v.c2 = 20;
            v.c3 = 13;
            v.c4 = -47;
            v.c5 = 12;
            v.c6 = -57;
            v.c7 = -59;
            v.c8 = 23;
            v.c9 = -38;
            v.c10 = -18;
            v.c11 = -12;



        }

        else
        {
            v = new Vars();
        }
        return v;
    }

    public static void main(String[] args) {

//        Vars v = GetVars(Mode.Left1);
//        Vars v = GetVars(Mode.Right1);
//        Vars v = GetVars(Mode.SpeedTest1);
        Vars v = GetVars(Mode.MiddleTestRight);

        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(v.startPose)

//region SpeedTest
//                                .lineToSplineHeading(new Pose2d(v.c1, v.c2, Math.toRadians(180)))
//                                .splineToConstantHeading(new Vector2d(v.c3, v.c4), Math.toRadians(0))
//endregion

//region MiddleManTest
                                .lineToSplineHeading(new Pose2d(v.c1, v.c2, Math.toRadians(v.d1)))
                                .lineToSplineHeading(new Pose2d(v.c1, v.c3, Math.toRadians(v.d2)))
                                .splineToConstantHeading(new Vector2d(v.c4, v.c5), Math.toRadians(v.d2))
//endregion

                                //region Right 1
//                                .lineToSplineHeading(new Pose2d(v.c6, v.c5, Math.toRadians(v.d3)))
//                                .splineToConstantHeading(new Vector2d(v.c7, v.c8), Math.toRadians(v.d1))
                                //endregion

                                //region Right 2
//                                .lineToSplineHeading(new Pose2d(v.c9, v.c5, Math.toRadians(v.d1)))
//                                .splineToConstantHeading(new Vector2d(v.c1,v.c8), Math.toRadians(v.d1))
                                //endregion

                                //region Right 3
                                .lineToSplineHeading(new Pose2d(v.c10, v.c5, Math.toRadians(v.d2)))
                                .lineToSplineHeading(new Pose2d(v.c11, v.c5, Math.toRadians(v.d1)))
                                .splineToConstantHeading(new Vector2d(v.c11, v.c8), Math.toRadians(v.d1))
                                //endregion

                                .build()
                );

        myBot.pause();
        myBot.setDimensions(12, 18);

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(180.95f)
                .addEntity(myBot)
                .start();
    }
}

