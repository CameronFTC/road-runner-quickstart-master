package org.firstinspires.ftc.teamcode.ultimategoal2020;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "trigTeleop", group = "TeleOp")
public class trigTeleop extends LinearOpMode {

    //define motors, servos, sensors, variables, etc.

    public double rightstickx;
    public double leftstickx;
    public double leftstickyfront;
    public double leftstickyback;
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;

    @Override
    public void runOpMode() {

            //hardware map motors, servos, etc.
            init();
            fL = hardwareMap.dcMotor.get("fL");
            fR = hardwareMap.dcMotor.get("fR");
            bL = hardwareMap.dcMotor.get("bL");
            bR = hardwareMap.dcMotor.get("bR");

            // everything that u want to happen after hit start button
            waitForStart();

            while (opModeIsActive()) {


            }

    }


    public void trigMecanum() {

        rightstickx = Math.abs(gamepad1.right_stick_x) * -gamepad1.right_stick_x ;
        leftstickx = -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);

        leftstickyfront = Math.abs(gamepad1.left_stick_y) * gamepad1.left_stick_y ;
        leftstickyback = Math.abs(gamepad1.left_stick_y) * -gamepad1.left_stick_y ;

        double rFront = Math.hypot(rightstickx, leftstickyfront);
        double rBack = Math.hypot(rightstickx, leftstickyback);

        double robotAngleFront = Math.atan2(leftstickyfront, rightstickx) - Math.PI / 4;
        double robotAngleBack = Math.atan2(leftstickyback, rightstickx) - Math.PI / 4;

        double rightX = leftstickx;

        final double v1 = rFront * Math.cos(robotAngleFront) + rightX;
        final double v2 = rFront * Math.sin(robotAngleFront) - rightX;
        final double v3 = rBack * Math.sin(robotAngleBack) + rightX;
        final double v4 = rBack * Math.cos(robotAngleBack) - rightX;

        /*
        telemetry.addData("fl", v1);
        telemetry.addData ("fR", v2);
        telemetry.addData ("bL", v3);
        telemetry.addData ("bR", v4);
        telemetry.addData("leftX", gamepad1.left_stick_x);
        telemetry.addData("leftY", gamepad1.left_stick_y);
        telemetry.addData ("Right X", rightX);
        telemetry.update();
        */

        fL.setPower(-v1);
        fR.setPower(-v2);
        bL.setPower(v3);// * .79);
        bR.setPower(v4);// * .79);
    }

}
