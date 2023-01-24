package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.ImageDetectorPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.ImageDetectorPipelineArea;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "CycleRight", group="Bertha", preselectTeleOp = "BerthaTeleOp")
public class AutoCycleRight extends LinearOpMode{

    enum State{
        PlacingCone,
        OpeningClaw,
        PreCone,
        ClawDrop, None
    }

    @Override
    public void runOpMode() throws InterruptedException {

//        String pipelineColorSeen = pipeline.ColorSeen;
        //Initializes Bertha Autonomous with the State to right for the right side
        BerthaAuto bertha = new BerthaAuto(hardwareMap, telemetry, BerthaAuto.AutoState.Right);
        bertha.AutoCheck();

        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        bertha.Read();
        timer.startTime();

        //TODO MAKE SURE TO UNBLOCK THIS TO ALLOW CYCLES TO HAPPEN
        //region UNBLOCK THIS TO ALLOW CYCLES
        try {
            //Drive to Cones
            bertha.DriveToConeStation(AutonomousDrive.DriveSpeed.Conservative);

            //Cycle Cones
            bertha.CycleCones(timer, 25);
        }
        catch(Exception ex){
            telemetry.addData("Exception: ", ex.getMessage());
            telemetry.update();
            throw new InterruptedException();
        }
        //Cycle Bertha to default state
        bertha.CycleDown();
        //endregion

        //Park Bertha
        bertha.Park();
    }
}