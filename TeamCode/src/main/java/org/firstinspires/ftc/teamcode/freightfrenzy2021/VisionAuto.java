package org.firstinspires.ftc.teamcode.freightfrenzy2021;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ultimategoal2020.WebCamVision;

@Autonomous (name = "VisionAuto", group = "Autonomous")
public class VisionAuto extends LinearOpMode {
    //hardwareMap robot = new hardwareMap();
    public ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        wcVision vision = new wcVision(this);
        //robot.init(this);

        waitForStart();

        while(opModeIsActive()){
            String rgb = vision.rbgVals(689,297);
            telemetry.addData("RGB: ", rgb);
            telemetry.update();
        }
    }
}

