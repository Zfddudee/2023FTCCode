package org.firstinspires.ftc.teamcode.drive.opmode.Vision;

import org.opencv.core.Scalar;

public class ColorRange {
    public Scalar Lower;
    public Scalar Upper;
    public String Color;

    public ColorRange(String color, Scalar lower, Scalar upper){
        Color = color;
        Lower = lower;
        Upper = upper;
    }
}
