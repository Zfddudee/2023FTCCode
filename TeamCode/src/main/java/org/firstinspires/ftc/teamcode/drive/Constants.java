package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    TeleopMain2023 Cases = new TeleopMain2023();

    //Boolean for if stomp is available to raise/lower
    public static boolean readyToStomp = true;
    //Boolean for if odometry is available to raise/lower
    public static boolean odoReady = true;
    //Boolean for weather or not turret is turning to the left
    public static boolean Left = false;

    //Values for velocities of motors
    public static final int HighVelocity = 2700;
    public static final int LowVelocity = 600;

    //Stomp servo values
    public static final double StompDown = 0.5;
    public static final double StompUp = 0;

    //Odometry servo values
    public static final double OdoUp = 0.5;
    public static final double OdoDown = 0;

    //Turret servo values
    public static final double TurretRight = 0.1;
    public static final double TurretLeft = 0.8;
    public static final double TurretDefault = 0.35;

    //Claw servo values
    public static final double ClawClosed = 0;
    public static final double ClawOpen = 0.9;

    //Claw scale range
    public static final double ClawLowRange = 0.2;
    public static final double ClawHighRange = 0.5;

    //Extake servo flip values
    public static final double ExtakeFlipIn = 1;
    public static final double ExtakeFlipOut = 0.3;
    public static final double ExtakeFlipLow =  0.5; //TODO Varify this variable is accurate

    //Extake servo slide values
    public static final double SlideOut = 0;
    public static final double SlideIn = 1;

    //Intake servo flip values
    public static final double ServoIntakeFlipIntaking = 0.9;
    public static final double ServoIntakeFlipExchanging = 0.2;

    //Intake slide motor values
    public static final int IntakeOut = 790;
    public static final int IntakeExchanging = 500;
    public static final int IntakeIn = 0;

    //Intake flip motor values
    public static final int IntakeFlips = 550;
    public static final int IntakeFlipsLow = 350;
    public static final int IntakeFlipsIn = 0;

    //Lift motor values
    public static final int LiftHigh = 1150;
    public static final int LiftMid = 400;
    public static final int LiftLow = 0;

    //Intake wheels
    public static final double IntakeWheelsIn = 0.5;
    public static final double IntakeWheelsOut = -1.0;

}
