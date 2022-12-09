package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Test", group="Testing")
public class test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Bertha bertha = new Bertha(hardwareMap, telemetry);
        boolean flag = true;
        waitForStart();
        while (!isStopRequested()) {
//            if(flag) {
//                bertha.IntakeFlipUp();
//                flag = false;
//            }
//            else
//            {
//                bertha.IntakeFlipDown();
//                flag = true;
//            }
            bertha.IntakeFlipDown();
            telemetry.update();
            sleep(5000);
        }
    }
}
