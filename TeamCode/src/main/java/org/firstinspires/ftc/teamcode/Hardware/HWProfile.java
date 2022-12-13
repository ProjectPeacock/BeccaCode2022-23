package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HWProfile {
    //constants
    public final boolean fieldCentric=true;

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
    public MotorEx motorLF = null;
    public MotorEx motorLR = null;
    public MotorEx motorRF = null;
    public MotorEx motorRR = null;
    public MotorEx motorLiftFront = null;
    public MotorEx motorLiftRear = null;
    public RevIMU imu = null;
    public ServoEx servoGrabber = null;
    public MecanumDrive mecanum = null;
    public MotorGroup winchMotors = null;


    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private final ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HWProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //drive motor init
        MotorEx motorLF = new MotorEx(ahwMap, "motorLF", Motor.GoBILDA.RPM_1150);
        motorLF.setRunMode(Motor.RunMode.PositionControl);
        motorLF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLF.resetEncoder();

        MotorEx motorLR = new MotorEx(ahwMap, "motorLR", Motor.GoBILDA.RPM_1150);
        motorLR.setRunMode(Motor.RunMode.PositionControl);
        motorLR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLR.resetEncoder();

        MotorEx motorRF = new MotorEx(ahwMap, "motorRF", Motor.GoBILDA.RPM_1150);
        motorRF.setRunMode(Motor.RunMode.PositionControl);
        motorRF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRF.resetEncoder();

        MotorEx motorRR = new MotorEx(ahwMap, "motorRR", Motor.GoBILDA.RPM_1150);
        motorRR.setRunMode(Motor.RunMode.PositionControl);
        motorRR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRR.resetEncoder();

        //lift motor init
        MotorEx motorLiftFront = new MotorEx(ahwMap, "motorLiftFront", Motor.GoBILDA.RPM_1150);
        motorLiftFront.setRunMode(Motor.RunMode.PositionControl);
        motorLiftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        MotorEx motorLiftRear = new MotorEx(ahwMap, "motorLiftFront", Motor.GoBILDA.RPM_1150);
        motorLiftRear.setRunMode(Motor.RunMode.PositionControl);
        motorLiftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //drivebase init
        MecanumDrive mecanum = new MecanumDrive(motorLF, motorLR, motorRF, motorRR);

        //establish motorgroup for lift
        MotorGroup winchMotors = new MotorGroup (motorLiftFront, motorLiftRear);

        //init servos
        ServoEx servoGrabber = new SimpleServo(ahwMap,"servoGrabber",0.3,0.55, AngleUnit.RADIANS);

        //init imu
        RevIMU imu = new RevIMU(ahwMap);
        imu.init();
    }
}  // end of HWProfile Class