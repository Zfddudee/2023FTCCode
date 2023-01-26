package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.ImageDetectorPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "CycleLeft", group="Bertha", preselectTeleOp = "BerthaTeleOp")
public class AutoCycleLeft extends LinearOpMode{


    @Override
    public void runOpMode() throws InterruptedException {
        //Initializes Bertha Autonomous with the State to right for the right side
        BerthaAuto bertha = new BerthaAuto(hardwareMap, telemetry, BerthaAuto.AutoState.Left);
        bertha.IsCycleLeft = true;
        bertha.AutoCheck();

        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        bertha.Auto();
        bertha.Read();
        timer.startTime();
        timer.reset();

        //Drive to Cones
        bertha.DriveToConeStation(AutonomousDrive.DriveSpeed.Conservative);

        bertha.PlaceCones(timer, 25);

//        //Cycle Bertha to default state
        bertha.CycleDown();
        //Park Bertha
        bertha.Park();

    }

}
