package org.firstinspires.ftc.teamcode.drive.opmode.Vision;

import com.vuforia.Area;

import org.opencv.core.Scalar;

public class ColorRange {
    public Scalar Lower;
    public Scalar Upper;
    public String Color;
    public double AreaSeen;

    public ColorRange(String color, Scalar lower, Scalar upper){
        Color = color;
        Lower = lower;
        Upper = upper;
        AreaSeen = 0.0;
    }
}
