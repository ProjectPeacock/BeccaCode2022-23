package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HWProfile {
    //constants
    public final boolean fieldCentric=true;

    //claw positions
    public final double CLAW_OPEN =0.3;
    public final double CLAW_CLOSE =0.55;

    //drive constants
    public final double MAX_DRIVE_POWER =1;
    public final double TURN_MULTIPLIER = 0.75;
    public final double STRAFE_FACTOR = 0.75;

    //anti-tip constants
    public final double ANTI_TIP_ADJ=0.3;
    public final double ANTI_TIP_TOL=10;
    public final int ANTI_TIP_AXIS=1;

    final public double DRIVE_TICKS_PER_INCH = 23.7;
    final public double USD_COUNTS_PER_INCH = 23.7;

    //lift constants
    final public double LIFT_POW=1;
    final public int MAX_LIFT_VALUE = 1275;
    final public int LIFT_BOTTOM=0;
    final public int LIFT_LOW=400;
    final public int LIFT_MID=850;
    final public int LIFT_HIGH=1200;
    final public double LIFT_KP=0.0225;
    final public int LIFT_TOL=10;

    final public double PARK_TIME = 27;     // sets the time for when the robot needs to park in auto

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
    public MotorEx autoLight = null;
    public Motor.Encoder forwardBackwardOdo = null;
    public Motor.Encoder sideSideOdo = null;

    public MotorGroup winch = null;

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
        motorLF = new MotorEx(ahwMap, "motorLF", Motor.GoBILDA.RPM_1150);
        motorLF.setRunMode(Motor.RunMode.RawPower);
        motorLF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLF.resetEncoder();

        motorLR = new MotorEx(ahwMap, "motorLR", Motor.GoBILDA.RPM_1150);
        motorLR.setRunMode(Motor.RunMode.RawPower);
        motorLR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLR.resetEncoder();

        motorRF = new MotorEx(ahwMap, "motorRF", Motor.GoBILDA.RPM_1150);
        motorRF.setRunMode(Motor.RunMode.RawPower);
        motorRF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRF.resetEncoder();

        motorRR = new MotorEx(ahwMap, "motorRR", Motor.GoBILDA.RPM_1150);
        motorRR.setRunMode(Motor.RunMode.RawPower);
        motorRR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRR.resetEncoder();

        //drivebase init
        mecanum = new MecanumDrive(motorLF, motorRF, motorLR, motorRR);

        //lift motors init
        motorLiftFront = new MotorEx(ahwMap, "motorLiftFront", Motor.GoBILDA.RPM_1150);
        motorLiftRear = new MotorEx(ahwMap, "motorLiftRear", Motor.GoBILDA.RPM_1150);

        winch = new MotorGroup(motorLiftFront,motorLiftRear);
        winch.setPositionCoefficient(LIFT_KP);
        winch.setTargetPosition(0);
        winch.setRunMode(Motor.RunMode.PositionControl);
        winch.setPositionTolerance(LIFT_TOL);
        winch.setPositionCoefficient(LIFT_KP);
        winch.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        winch.stopMotor();
        winch.resetEncoder();

        //light init
        autoLight = new MotorEx(ahwMap, "sideSideOdo", 8192,10000);
        autoLight.set(0);

        //odometry encoders init
        sideSideOdo = autoLight.encoder;
        sideSideOdo.reset();

        MotorEx xAxisOdoPlaceholder = new MotorEx(ahwMap, "forwardBackwardOdo", 8192,10000);
        forwardBackwardOdo = xAxisOdoPlaceholder.encoder;
        forwardBackwardOdo.reset();

        //init servos
        servoGrabber = new SimpleServo(ahwMap,"servoGrabber",0.3,0.55, AngleUnit.RADIANS);

        //init imu
        imu = new RevIMU(ahwMap);
        imu.init();
    }
}  // end of HWProfile Class