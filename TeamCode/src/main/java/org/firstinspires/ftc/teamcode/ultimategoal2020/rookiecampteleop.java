package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime; 

@TeleOp(name = "Rookie Op Mode", group = "TeleOp")
public class rookiecampteleop extends LinearOpMode { //changed from "OpMode" to "LinearOpMode" (LinearOpMode alr extends OpMode)
    DcMotor leftMotorT;
    DcMotor leftMotorB;
    DcMotor rightMotorT;
    DcMotor rightMotorB;
    double leftPowerT; //you don't need to make variables for power --> set pwr ex. leftMotorT.setPower(power)
    double leftPowerB;
    double rightPowerT;
    double rightPowerB;
    double leftStickY = -gamepad1.left_stick_y;
    double rightStickX  =  gamepad1.right_stick_x;
    double leftStickX = gamepad1.left_stick_x; //u made this trigger, but i think it makes more sense to do left stick x
    double rightTrigger = gamepad1.right_trigger;
    double leftTrigger = gamepad1.left_trigger;

    ElapsedTime time = new ElapsedTime(); //can delete this (don't rly use time anywhere)

    @Override
    public void runOpMode() { //everything that happens in init & after start needs to be in a runOpMode loop
        init();
        leftMotorT = hardwareMap.dcMotor.get("leftDriveT"); //i would keep leftDriveT as leftMotorT for consistency
        leftMotorB = hardwareMap.dcMotor.get("leftDriveB");
        rightMotorT = hardwareMap.dcMotor.get("rightDriveT");
        rightMotorB = hardwareMap.dcMotor.get("rightDriveB");


        waitForStart(); //this is the start command --> everything under here will happen once u click the start button


        while (opModeIsActive()) { //stops loop when driver hits stop (don't need stop command)
            leftMotorT.setPower(0); //ex. of how to set power (change other 3)
            leftPowerB = 0;
            rightPowerT = 0;
            rightPowerB = 0;

            //Move forwards and backwards
            if (leftStickY != 0) { //create dead zone --> if (leftStickY > 0.05 || leftStickY < -0.05)
                leftMotorT.setPower(leftStickY); //change other 3
                leftPowerB = leftStickY;
                rightPowerT = leftStickY;
                rightPowerB = leftStickY;
            }

            //Turn left and right
            else if (rightStickX != 0) { //create dead zone again (so that joysticks aren't too sensitive)
                leftPowerT = -rightStickX; //change to leftMotorT.setPower(-rightStickX) + other 3
                leftPowerB = -rightStickX;
                rightPowerT = rightStickX;
                rightPowerB = rightStickX;
            }

            //Strafe right & left
            else if (leftStickX > 0.05 || leftStickX < -0.05) { //changed from bumper to left joystick x axis
                leftMotorT.setPower(leftStickX);
                leftMotorB.setPower(-leftStickX);
                rightMotorT.setPower(-leftStickX);
                rightMotorB.setPower(leftStickX);
            }


            //Press Y to move diagonally backwards
            else if (gamepad1.y) {
                if (gamepad1.right_trigger > 0) {
                    leftPowerB = -rightTrigger;
                    rightPowerT = -rightTrigger;
                } else if (gamepad1.left_trigger > 0) {
                    leftPowerT = -leftTrigger;
                    rightPowerB = -leftTrigger;
                }
            }

            //Move diagonally right
            else if (gamepad1.right_trigger > 0) {
                leftPowerT = rightTrigger;
                rightPowerB = rightTrigger;
            }

            //Move diagonally left
            else if (gamepad1.left_trigger > 0) {
                leftPowerT = -leftTrigger;
                rightPowerB = -leftTrigger;
            }

            leftMotorT.setPower(leftPowerT);
            leftMotorB.setPower(leftPowerB);
            rightMotorT.setPower(rightPowerT);
            rightMotorB.setPower(rightPowerB);
        }
    }


}
