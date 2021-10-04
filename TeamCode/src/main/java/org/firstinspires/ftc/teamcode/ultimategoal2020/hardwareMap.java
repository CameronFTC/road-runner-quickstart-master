package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class hardwareMap {

    //Halfpower
    public double halfPower;

    //Sticks
    public double rightstickx;
    public double leftstickx;
    public double leftstickyfront;
    public double leftstickyback;

    // Motors
    public DcMotor fL = null;
    public DcMotor fR = null;
    public DcMotor bL = null;
    public DcMotor bR = null;
    public DcMotor intake = null;
    public DcMotor shooter = null;
    public DcMotor shooter2 = null;
    public DcMotor pulley1 = null;

    // Servos
    //public Servo clip = null;
    public Servo wobble = null;
    public CRServo elevator = null;
    public CRServo pusher = null;
    public CRServo brush = null;

    //Sensors
    //public TouchSensor touch = null;
    DigitalChannel touch;
    DigitalChannel highTouch;
    DigitalChannel powerTouch;

    // HardwareMap
    HardwareMap hwMap;

    // Linear Opmode
    LinearOpMode opmode;

    // Time
    public ElapsedTime runtime = new ElapsedTime();

    // Tick Conversion
    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public Orientation startPos = null;
    public double lastDegrees;
    public double globalAngle;
    public double referenceAngle;

    public double leftCorrect;
    public double rightCorrect;

    public double degreesToTicks;


    // Initialize Components
    public void init(LinearOpMode lOpmode) {
        opmode = lOpmode;
        // Hardware map
        hwMap = opmode.hardwareMap;

        degreesToTicks = 0; //add in actual conversion

        // Define and Initialize Motors
        fL = opmode.hardwareMap.get(DcMotor.class, "fL");
        fR = opmode.hardwareMap.get(DcMotor.class, "fR");
        bL = opmode.hardwareMap.get(DcMotor.class, "bL");
        bR = opmode.hardwareMap.get(DcMotor.class, "bR");
        intake = opmode.hardwareMap.get(DcMotor.class, "intake");
        shooter = opmode.hardwareMap.get(DcMotor.class, "shooter");
        shooter2 = opmode.hardwareMap.get(DcMotor.class, "shooter2");
        pulley1 = opmode.hardwareMap.get(DcMotor.class, "pulley");
        wobble = opmode.hardwareMap.get(Servo.class, "wobbleGrabber");
        elevator = opmode.hardwareMap.get(CRServo.class, "elevator");
        pusher = opmode.hardwareMap.get(CRServo.class, "pusher");
        brush = opmode.hardwareMap.get(CRServo.class, "brush");
        touch = opmode.hardwareMap.get(DigitalChannel.class, "touch");
        highTouch = opmode.hardwareMap.get(DigitalChannel.class, "highTouch");
        powerTouch = opmode.hardwareMap.get(DigitalChannel.class, "powerTouch");

        degreesToTicks = 560.0 / 360.0;

        //set direction of motors
        fL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);

        //intake.setDirection(DcMotor.Direction.FORWARD);
        //shooter.setDirection(DcMotor.Direction.FORWARD);

        //initColor();
        //intakeL.setDirection(DcMotor.Direction.rotate);
        //intakeR.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders, and run without encoders
        reset();

        // Set motor powers to zero
        stopMotors();


        // Set all motors to zero power
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = opmode.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        //driveTrain.srvMarker.setPosition(1);


        opmode.telemetry.addData("Mode", "calibrating...");
        opmode.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!opmode.isStopRequested() && !imu.isGyroCalibrated()) {
            opmode.sleep(50);
            opmode.idle();
        }

        opmode.telemetry.addData("Mode", "waiting for start");
        opmode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        opmode.telemetry.update();

        //opmode.telemetry.addData("fl", fL.getCurrentPosition());
        //opmode.telemetry.addData("fr", fR.getCurrentPosition());
        //opmode.telemetry.addData("bl", bL.getCurrentPosition());
        //opmode.telemetry.addData("br", bR.getCurrentPosition());
        //opmode.telemetry.update();

    }

    public void init(LinearOpMode lOpmode, Boolean teleop) {
        opmode = lOpmode;

        halfPower = 1;
        // Hardware map
        hwMap = opmode.hardwareMap;

        // Define and Initialize Motors
        fL = opmode.hardwareMap.get(DcMotor.class, "fL");
        fR = opmode.hardwareMap.get(DcMotor.class, "fR");
        bL = opmode.hardwareMap.get(DcMotor.class, "bL");
        bR = opmode.hardwareMap.get(DcMotor.class, "bR");
        shooter = opmode.hardwareMap.get(DcMotor.class, "shooter");
        shooter2 = opmode.hardwareMap.get(DcMotor.class, "shooter2");
        intake = opmode.hardwareMap.get(DcMotor.class, "intake");
        pulley1 = opmode.hardwareMap.get(DcMotor.class, "pulley");

        //Define and initialize servos
        //clip = hwMap.get(Servo.class, "clip");
        wobble = opmode.hardwareMap.get(Servo.class, "wobbleGrabber");
        elevator = opmode.hardwareMap.get(CRServo.class, "elevator");
        pusher = opmode.hardwareMap.get(CRServo.class, "pusher");
        brush = opmode.hardwareMap.get(CRServo.class, "brush");

        touch = opmode.hardwareMap.get(DigitalChannel.class, "touch");
        highTouch = opmode.hardwareMap.get(DigitalChannel.class, "highTouch");
        powerTouch = opmode.hardwareMap.get(DigitalChannel.class, "powerTouch");

        //set direction of motors
        fL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders, and run without encoders
        reset();

        // Set motor powers to zero
        stopMotors();

        // Set all motors to zero power
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        opmode.telemetry.addData("fl", fL.getCurrentPosition());
        opmode.telemetry.addData("fr", fR.getCurrentPosition());
        opmode.telemetry.addData("bl", bL.getCurrentPosition());
        opmode.telemetry.addData("br", bR.getCurrentPosition());
        opmode.telemetry.addData("robot", " initialized");
        opmode.telemetry.update();

    }
    public void stopMotors(){
        fL.setPower(0);
        bL.setPower(0);
        fR.setPower(0);
        bR.setPower(0);
    }
    public void reset() {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);    //without encoder before
        opmode.idle();
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        opmode.idle();
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        opmode.idle();
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        opmode.idle();

    }
}
