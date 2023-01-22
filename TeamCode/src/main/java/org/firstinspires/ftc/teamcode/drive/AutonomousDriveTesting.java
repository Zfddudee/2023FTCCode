package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous Drive Testing", group="Bertha Testing", preselectTeleOp = "BerthaTeleOp")
@Config
public class AutonomousDriveTesting extends LinearOpMode {

    public AutonomousDrive.DriveSpeed speed = AutonomousDrive.DriveSpeed.Conservative;
    public AutonomousDrive.DriveDirection direction = AutonomousDrive.DriveDirection.Right;

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousDrive drive = new AutonomousDrive(hardwareMap);
        drive.Go(speed, direction);
    }
}
