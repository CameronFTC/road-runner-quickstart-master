package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

@Autonomous(name="rookiecampauto", group="Autonomous")

public class rookiecampauto extends LinearOpMode {

    ModernRoboticsI2cGyro   gyro    = null;
    ElapsedTime time = new ElapsedTime();

    //define variables, motors, etc.

    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    Servo dump;
    SensorColor colorSensor;

    @Override
    public void runOpMode() {

        //stuff that happens after init is pressed
        init();
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        fL = hardwareMap.dcMotor.get("fL");
        fR = hardwareMap.dcMotor.get("fR");
        bL = hardwareMap.dcMotor.get("bL");
        bR = hardwareMap.dcMotor.get("bR");

        telemetry.addLine("go team");
        telemetry.addLine("hi");
        telemetry.addData("fL position:", fL.getCurrentPosition());
        telemetry.update();

        gyro.calibrate();

        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        //stuff that happens after start is pressed
        waitForStart();

        while (opModeIsActive()) {

            //call functions

            go_straight(500, -0.5);
            // strafe left
            // turn right
            dump.setPosition(1);
            // go backwards
            // strafe left


        }

    }

    //create functions

    public void go_straight (double distance, double pwr) {
        double startVal = fL.getCurrentPosition();

        while ((Math.abs(fL.getCurrentPosition() - startVal) < distance) && opModeIsActive()) {
            fL.setPower(pwr);
            fR.setPower(pwr);
            bL.setPower(pwr);
            bR.setPower(pwr);
        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public void go_straight_gyro (double distance, double pwr) {
        gyro.resetZAxisIntegrator();
        double startVal = fL.getCurrentPosition();

        while ((Math.abs(fL.getCurrentPosition() - startVal) < distance) && opModeIsActive()) {
            if (gyro.getIntegratedZValue() > 1) {
                fL.setPower(pwr * 0.9);
                fR.setPower(pwr);
                bL.setPower(pwr * 0.9);
                bR.setPower(pwr);
            }

            else if (gyro.getIntegratedZValue() < 1) {
                fL.setPower(pwr);
                fR.setPower(pwr * 0.9);
                bL.setPower(pwr);
                bR.setPower(pwr * 0.9);
            }

            else {
                fL.setPower(pwr);
                fR.setPower(pwr);
                bL.setPower(pwr);
                bR.setPower(pwr);
            }
        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public void strafe_right (double distance, double pwr) {
        double startVal2 = fL.getCurrentPosition();

        while((Math.abs(fL.getCurrentPosition() - startVal2) < distance) && opModeIsActive()) {
            fL.setPower(pwr);
            fR.setPower(-pwr);
            bL.setPower(-pwr);
            bR.setPower(pwr);

        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public void strafe_right_gyro (double distance, double pwr) {
        gyro.resetZAxisIntegrator();
        double startVal2 = fL.getCurrentPosition();
    
        while((Math.abs(fL.getCurrentPosition() - startVal2) < distance) && opModeIsActive()) {
            if (gyro.getIntegratedZValue() > 1) {
                fL.setPower(pwr * 0.9);
                fR.setPower(-pwr);
                bL.setPower(-pwr * 0.9);
                bR.setPower(pwr);
            }

            else if (gyro.getIntegratedZValue() < 1) {
                fL.setPower(pwr);
                fR.setPower(-pwr * 0.9);
                bL.setPower(-pwr);
                bR.setPower(pwr * 0.9);
            }

            else {
                fL.setPower(pwr);
                fR.setPower(-pwr);
                bL.setPower(-pwr);
                bR.setPower(pwr);
            }

        }
    
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }
    
    public void turnBasic_right(double turn, double pwr) {
        gyro.resetZAxisIntegrator();

        while ((gyro.getIntegratedZValue() < turn) && opModeIsActive()) {
                fL.setPower(pwr);
                fR.setPower(-pwr);
                bL.setPower(pwr);
                bR.setPower(-pwr);

        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public void turnBasic_left(double turn, double pwr) {
        gyro.resetZAxisIntegrator();

        while ((gyro.getIntegratedZValue() > turn) && opModeIsActive()) {
            fL.setPower(-pwr);
            fR.setPower(pwr);
            bL.setPower(-pwr);
            bR.setPower(pwr);

        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }
    
    public void turnPID(double angle, double pwr, double d) {
        gyro.resetZAxisIntegrator();
        time.reset();

        double deltaAngle = Math.abs(angle - gyro.getHeading());
        double pastdeltaAngle = deltaAngle;
        double currentTime;
        double kP = pwr / angle;
        double kI = 0.01;
        double kD = d / angle;
        double prevTime = 0;
        double apply = 0;
        double deltaTime;

        while (Math.abs(deltaAngle) > 1){
            deltaAngle = Math.abs(angle - gyro.getHeading());
            kP = deltaAngle * kP;
            currentTime = time.milliseconds();
            deltaTime =  currentTime - prevTime;
            kI = deltaAngle * deltaTime * kI;
            kD = (deltaAngle - pastdeltaAngle) / deltaTime * kD;
            apply = kP + kI + kD;

            fL.setPower(-apply);
            fR.setPower(apply);
            bL.setPower(-apply);
            bR.setPower(apply);

            prevTime = currentTime;
            pastdeltaAngle = deltaAngle;
        }
    }
}


