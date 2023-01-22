package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.JunctionPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.Vision.JunctionPipeline2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "BerthaTeleOp", group="Bertha")
public class BerthaTeleOp extends LinearOpMode {

    private Bertha bertha;
    private boolean rightTrigger = false;
    private boolean touchpad = false;
    private boolean ButtonA = false;
    private boolean ButtonY = false;
    private boolean ButtonX = false;
    private boolean ButtonB = false;
    private boolean Share = false;
    private boolean Gamepad2X = false;
    public double X1;
    public double X2;

    OpenCvCamera webcam;
    OpenCvCamera webcam2;
    JunctionPipeline pipeline;
    JunctionPipeline2 pipeline2;
    @Override
    public void runOpMode() throws InterruptedException {
        bertha = new Bertha(hardwareMap, telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FalconCam2"), cameraMonitorViewId);
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FalconCam"));
        pipeline = new JunctionPipeline();
        pipeline2 = new JunctionPipeline2();
        webcam.setPipeline(pipeline);
        webcam2.setPipeline(pipeline2);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                                         }


                                         public void onError(int errorCode) {

                                         }
                                     }

        );
        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                          @Override
                                          public void onOpened() {
                                              webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                                          }


                                          public void onError(int errorCode) {

                                          }
                                      }

        );
        waitForStart();
        while (!isStopRequested()) {
            X1 = pipeline.x;
            X2 = pipeline2.x;
            Constants.X1 = pipeline.x;
            Constants.X2 = pipeline2.x;
//            X1 = 230;
//            X2 = 230;
            try {
                GamePad1Loop();
                GamePad2Loop();
                bertha.RunOpMode();
            }
            catch (Exception ex)
            {
                telemetry.addData("ERROR in main loop", ex.getMessage());
                telemetry.addData("Inner exception", ex.getStackTrace());
            }
            telemetry.addData("X1", X1);
            telemetry.addData("X2", X2);
            telemetry.update();
        }
    }

    private void GamePad1Loop() {
        //This moves the robot forwards, backwards, and strafes left and right
        //Left Joystick
        bertha.Move(new Pose2d(
                -gamepad1.left_stick_y * -1 * Constants.DrivePower,
                -gamepad1.left_stick_x * -1 * Constants.DrivePower,
                -gamepad1.right_stick_x * Constants.DrivePower
        ));

        if(gamepad1.x && !ButtonX) {
            bertha.IntakeOpenClaw();
            ButtonX = true;
        }
        //opens intake claw
        else if(gamepad1.a && !ButtonA) {
                bertha.PreConePickup();
                ButtonA = true;
        }

        //Todo
        // Remove test program.
        //sets to going to pickup cone positions
//        else if(gamepad1.b)
//            bertha.CameraCenterTest();
        else if(gamepad1.y && !ButtonY) {
            bertha.PickUpOverRide();
            ButtonY = true;
        }
        //Brings arm back to overide if goes wrong
        else if(gamepad1.dpad_up)
            bertha.MoveIntake(-20);
        //manual move intake arm out
        else if(gamepad1.dpad_down)
            bertha.MoveIntake(20);
        //manual move intake arm in
        else if(gamepad1.dpad_right)
            bertha.MoveSlide(30);
        //manual move intake slide out
        else if(gamepad1.dpad_left)
            bertha.MoveSlide(-30);
        //manual move intake slide in
        else if(gamepad1.share && !Share) {
            bertha.Reset();
            Share = true;
        }
        //resets robot to moving state
        else if(gamepad1.right_bumper)
            bertha.StompDown();
        //puts stomp down
        else if(gamepad1.left_bumper)
            bertha.StompUp();
        //puts stomp up
        else if(gamepad1.touchpad && !touchpad) {
                bertha.MoveToExchange2();
                touchpad = true;
        }
        //started exchange proccess
//        else if(gamepad1.right_stick_button)

//            bertha.AutoExtake();

        //open/close claw toggle button
        else if(gamepad1.right_trigger != 0 && !rightTrigger) {
                bertha.OpenCloseIntakeClaw();
                rightTrigger = true;
        }
        else if(gamepad1.right_trigger == 0)
            rightTrigger = false;
        //flips intake wrist flip
        else if(gamepad1.right_stick_button)
            bertha.ToggleClawFlip();

        if (!gamepad1.touchpad)
            touchpad = false;
        if (!gamepad1.a)
            ButtonA = false;
        if (!gamepad1.b)
            ButtonB = false;
        if (!gamepad1.x)
            ButtonX = false;
        if (!gamepad1.y)
            ButtonY = false;
        if (!gamepad1.share)
            Share = false;
    }

//todo fix gamepad 2 controls to be toggles
    private void GamePad2Loop() {
        if(gamepad2.a)
            bertha.TeleOpCycle();
        else if(gamepad2.b)
            bertha.IntakeReturn();
        if(gamepad2.y)
            bertha.LiftMedium();
        //opens extake claw
        else if(gamepad2.right_trigger > 0)
            bertha.OpenClaw();
        //closes extake claw
        else if(gamepad2.left_trigger > 0)
            bertha.CloseClaw();
        //moves turret to right position
        else if(gamepad2.right_bumper)
            bertha.TurretRight();
        //moves turret to left position
        else if(gamepad2.left_bumper)
            bertha.TurretLeft();
//        else if(gamepad2.right_stick_y != 0)
//            bertha.MoveLiftOffset(gamepad2.right_stick_y);
        //resets robot to moving position
        else if(gamepad2.share)
            bertha.Reset();
        else if(gamepad2.dpad_up)
            bertha.LiftPositioning(1);
        else if(gamepad2.dpad_down)
            bertha.LiftPositioning(-1);
            //centers turret
        else if(gamepad2.touchpad)
            bertha.TurretCenter();
            //toggle between extake slide positions
        else if(gamepad2.x) {
            if (!Gamepad2X) {
                bertha.ExtakeSlideInOut();
                Gamepad2X = true;
            }
        }
        else if(!gamepad2.x)
            Gamepad2X = false;

        //manually move arm
        if(gamepad2.left_stick_y != 0)
            bertha.TurretVertical(Constants.TurretVerticalStepOver * gamepad2.left_stick_y * Constants.TurretVertialSpeedMultiplier);
        //manually move turret
        if(gamepad2.right_stick_x != 0)
            bertha.TurretHorizontal(Constants.TurretStepOver * gamepad2.right_stick_x * Constants.TurretHorizontalSpeedMultiplier);

//        else if(gamepad2.left_stick_button)
//            bertha.ExchangeToExtake();


    }
}
