package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "WebCamAuto", group = "Autonomous")
public class WebCamAuto extends LinearOpMode {
    //hardwareMap robot = new hardwareMap();
    public ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        WebCamVision vision = new WebCamVision(this);
        //robot.init(this);

        waitForStart();

        while(opModeIsActive()){
            String rgb = vision.rbgVals(689,297);
            telemetry.addData("RGB: ", rgb);
            telemetry.update();
        }
    }
}
