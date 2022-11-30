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

    //Velocity for the slide vertical movement
    public static int HighVelocity = 2700;
    public static int LowVelocity = 600;

    //Stomp servo values
    public static double StompDown = 0.5;
    public static double StompUp = 0;

    //Odometry servo values
    public static double OdoUp = 0.5;
    public static double OdoDown = 0;

    //Turret servo values
    public static double TurretRight = 0.05;
    public static double TurretLeft = 1;
    public static double TurretDefault = 0.45;
    public static double TurretStepOver = -0.0035;

    //Claw servo values
    public static double ClawClosed = 0.26;
    public static double ClawOpen = 0.375;

    //Extake servo flip values
    public static double ExtakeFlipIn = 0;
    public static double ExtakeFlipIn2 = 1;
    public static double ExtakeFlipOut = 1;
    public static double ExtakeFlipOut2 = 0;
    public static double ExtakeFlipLow = 0.4;
    public static double ExtakeFlipLow2 = 0.6;
    public static double ExtakeFlipCycle = 0.7;
    public static double ExtakeFlipCycle2 = 0.3;


    //Extake servo slide values
    public static double SlideOut = 0;
    public static double SlideOut2 = 1;
    public static double SlideIn = 1;
    public static double SlideIn2 = 0;

    //Intake servo flip values
    public static double ServoIntakeFlipIntaking = 1.0;
    public static double ServoIntakeFlipExchanging = 0.27;

    //Intake slide motor values
    public static int IntakeOut = -670;
    public static int IntakeExchanging = -500;
    public static int IntakeIn = 0;

    //Intake flip motor values
    public static int IntakeFlips = 550;
    public static int IntakeFlipsLow = 400;
    public static int IntakeFlipsIn = 0;

    //Lift motor values
    public static int LiftHigh = -1000;
    public static int LiftMid = -500;
    public static int LiftLow = -200;
    public static int LiftDefault = 0;

    //Intake wheels
    public static double IntakeWheelsSlow = 0.25;
    public static double IntakeWheelsIn = 1;
    public static double IntakeWheelsOut = -1.0;
    public static double IntakeWheelStop = 0;
    public static double IntakeWheelSensor = 20;

    public static int TimeOutTime = 3000;

    public static double liftGainP = 0.002;
    public static double liftGainP0 = 0.0001;
    public static double liftGainPUp = 0.0025;
}
