package com.example.falconroboticstester;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Vars{

    public int d1, d2, d3, d4;
    public double c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18;

    public Pose2d startPose;
    public Pose2d endPose;

    public Vars(){
    }
    public void Init(int sX, int sY, int sRadians, int eX, int eY, int eRadians){
        startPose = new Pose2d(sX, sY, Math.toRadians(sRadians));
        endPose = new Pose2d(eX,eY, Math.toRadians(eRadians));
    }
}
