package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous Drive Testing", group="Bertha Testing", preselectTeleOp = "BerthaTeleOp")
@Config
public class AutonomousDriveTesting extends LinearOpMode {

    private AutonomousDrive.DriveSpeed speed;
    private AutonomousDrive.DriveDirection direction;

    public boolean IsConservative = true;
    public boolean IsRight = true;

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousDrive drive = new AutonomousDrive(hardwareMap);

        speed = (IsConservative)? AutonomousDrive.DriveSpeed.Conservative : AutonomousDrive.DriveSpeed.Fast;
        direction = (IsRight)? AutonomousDrive.DriveDirection.Right : AutonomousDrive.DriveDirection.Left;

        waitForStart();
        drive.Go(speed, direction);
        while(opModeIsActive()){
        }
    }
}
