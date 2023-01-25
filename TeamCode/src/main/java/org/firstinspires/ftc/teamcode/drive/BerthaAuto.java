package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.ImageDetectorPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.ImageDetectorPipelineArea;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.JunctionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class BerthaAuto extends Bertha {
    enum AutoState{
        Right,
        Left
    };

    private AutoState state;
    private int coneCount;
    private HardwareMap hardwareMap;
    private AutonomousDrive drive;

    private int last;

    private OpenCvCamera webcam;
    private ImageDetectorPipelineArea pipeline;

    public BerthaAuto (HardwareMap map, Telemetry telemetry, AutoState runState) {
        super(map, telemetry);
        hardwareMap = map;
        coneCount = 0;
        state = runState;
        InitCamAndPipeline();
    }

    private void InitCamAndPipeline(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FalconCam"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        pipeline = new ImageDetectorPipelineArea(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                                         }


                                         public void onError(int errorCode) {

                                         }
                                     }
        );
    }
    //parking position
    public void Read(){
       last = pipeline.Last;
    }

    public int GetParkPosition()
    {
        return last;
    }


    //Starting check on init to make sure subsytems positions correct.
    public void AutoCheck() {
        turret.CloseClaw();
        turret.SlideIn();
    }

    //Gets amount of cones placed
    public int GetConeCount(){
        return coneCount;
    }

    //Adds one to cone count
    public void IncrementCone(int step){
        coneCount += step;
    }

    //Main function to place all cones
    public void PlaceCones(ElapsedTime timer, int maxTime) {
        TeleOpCycle();
        intakeHeightOffset =  GetIntakeOffsetHeight(1);
        while(GetConeCount() <= Constants.ConeCount && timer.seconds() <= maxTime){
//            if(!IsCone())
//                drive.TurnCorrect();

//            if(extaking == Bertha.Extaking.TurretTurnLeft)
//                extaking = Bertha.Extaking.TurretTurnRight;
            RunOpMode();
//            if(extaking == Bertha.Extaking.None && lift.IsLiftAtPosition(Constants.LiftHigh, 200)){
            if(extaking == Bertha.Extaking.None && intaking == Intaking.None){
                IncrementCone(1);
                this.intakeHeightOffset = GetIntakeOffsetHeight(GetConeCount() + 1);
//                lift.Wait(500);
                IntakeReturn();
            }
        }
    }

    //Calls reset and resets everything for auto end
    public void CycleDown(){
        Reset();
        while(extaking == Bertha.Extaking.Reset || intaking == Bertha.Intaking.Reset)
        {
            RunOpMode();
        }
        turret.Wait(1000);
    }

    //Int position for intake arm heights
    private int GetIntakeOffsetHeight(int count){
        switch (count){
            case 1:
                return Constants.IntakeFlips1;

            case 2:
                return Constants.IntakeFlips2;

            case 3:
                return Constants.IntakeFlips3;

            case 4:
                return Constants.IntakeFlips4;

            case 5:
                return Constants.IntakeFlips5;

        }
        return 0;
    }

    //Function to call and drive to cones
    public void DriveToConeStation(AutonomousDrive.DriveSpeed speed){
        drive = new AutonomousDrive(hardwareMap);
        AutonomousDrive.DriveDirection direction = (state == AutoState.Right)? AutonomousDrive.DriveDirection.Right : AutonomousDrive.DriveDirection.Left;
        drive.Go(speed, direction);
    }

    //Function call to park in correct location
    public void Park(){
        ///TODO: add all logic to drive cone cycling to parking spot.
        drive.Park(GetParkPosition());
    }
}