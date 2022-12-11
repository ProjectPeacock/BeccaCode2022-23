package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HWProfile {
    //constants
    public final double CLAW_OPEN =0.3;
    public final double CLAW_CLOSE =0.55;

    public final double MAX_DRIVE_POWER =1;

    public final double BUTTON_TIMEOUT =0.5;

    public final double STRAFE_FACTOR = 0.75;

    final public double DRIVE_TICKS_PER_INCH = 23.7;
    final public double USD_COUNTS_PER_INCH = 23.7;

    final public int MAX_LIFT_VALUE = 5000;
    final public int MIN_LIFT_VALUE = 0;
    final public int JUNCTION_LOWER = 2000;
    final public int JUNCTION_MID = 4000;
    final public int JUNCTION_HIGH = 5000;

    final public double MIN_PIDROTATE_POWER = 0.2;

    /* Public OpMode members. */
    public DcMotorEx motorLF   = null;
    public DcMotorEx  motorLR  = null;
    public DcMotorEx  motorRF     = null;
    public DcMotorEx  motorRR    = null;
    public DcMotorEx motorLiftFront = null;
    public DcMotorEx motorLiftRear = null;
    public BNO055IMU imu = null;
    public Servo servoGrabber = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private final ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HWProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
//        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorDistance;

        // Define and Initialize Motors
        motorLF = hwMap.get(DcMotorEx.class, "motorLF");
        motorLF.setDirection(DcMotorEx.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorLF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLF.setPower(0);

        motorLR = hwMap.get(DcMotorEx.class, "motorLR");
        motorLR.setDirection(DcMotorEx.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorLR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLR.setPower(0);

        motorRF = hwMap.get(DcMotorEx.class, "motorRF");
        motorRF.setDirection(DcMotorEx.Direction.FORWARD);
        motorRF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRF.setPower(0);

        motorRR = hwMap.get(DcMotorEx.class, "motorRR");
        motorRR.setDirection(DcMotorEx.Direction.FORWARD);
        motorRR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRR.setPower(0);

        //lift motor init
        motorLiftFront = hwMap.get(DcMotorEx.class, "motorLiftRear");
        motorLiftFront.setDirection(DcMotorEx.Direction.FORWARD);
//        motorLiftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLiftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftFront.setTargetPosition(0);
        motorLiftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorLiftFront.setPower(0);

        motorLiftRear = hwMap.get(DcMotorEx.class, "motorLiftFront");
        motorLiftRear.setDirection(DcMotorEx.Direction.FORWARD);
//        motorLiftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLiftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftRear.setTargetPosition(0);
        motorLiftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorLiftRear.setPower(0);

        //init servos
        servoGrabber = hwMap.get(Servo.class, "servoGrabber");

        //init imu
        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

    }
}  // end of HWProfile Class