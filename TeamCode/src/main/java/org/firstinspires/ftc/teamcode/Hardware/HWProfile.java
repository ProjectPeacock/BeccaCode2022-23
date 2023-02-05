package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HWProfile {
    //constants
    public final boolean fieldCentric=true;

    //tflite file name
    public final String tfliteFileName = "PP_Generic_SS.tflite";

    //max difference between front and rear lift motors
    public final int rearLiftMotorTol = 8;

    //claw positions
    public final double CLAW_OPEN =0.3;
    public final double CLAW_CLOSE =0.55;

    //odometer positions
    public final double ODO_UP=0.25;
    public final double ODO_DOWN=0.95;

    //drive constants
    public final double TURN_MULTIPLIER = 0.75;

    //anti-tip constants
    public final double ANTI_TIP_ADJ=0.3;
    public final double ANTI_TIP_TOL=10;
    public final int ANTI_TIP_AXIS=1;

    //lift constants
    final public int liftAdjust=15;
    final public double LIFT_POW=1;
    final public int MAX_LIFT_VALUE = 1500;
    final public int LIFT_BOTTOM=0;
    final public int LIFT_LOW=525;
    final public int LIFT_MID=825;
    final public int LIFT_HIGH=1150;

    /* Public OpMode members. */
    public MotorEx motorLF = null;
    public MotorEx motorLR = null;
    public MotorEx motorRF = null;
    public MotorEx motorRR = null;
    public DcMotorEx motorLiftFront = null;
    public DcMotorEx motorLiftRear = null;
    public RevIMU imu = null;
    public ServoEx servoGrabber = null;
    public ServoEx odometryServo = null;
    public MecanumDrive mecanum = null;
    public MotorEx autoLight = null;

    /* local OpMode members. */
    HardwareMap hwMap =  null;

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
        motorLiftFront = hwMap.get(DcMotorEx.class, "motorLiftFront");
        motorLiftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftFront.setTargetPosition(0);
        motorLiftFront.setTargetPositionTolerance(10);
        motorLiftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLiftRear = hwMap.get(DcMotorEx.class, "motorLiftRear");
        motorLiftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftRear.setTargetPosition(0);
        motorLiftRear.setTargetPositionTolerance(rearLiftMotorTol);
        motorLiftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //light init
        autoLight = new MotorEx(ahwMap, "sideSideOdo", 8192,10000);
        autoLight.set(0);

        //init servos
        servoGrabber = new SimpleServo(ahwMap,"servoGrabber",0.3,0.55, AngleUnit.RADIANS);
        odometryServo = new SimpleServo(ahwMap,"odometryServo",0,1,AngleUnit.RADIANS);
        //init imu
        imu = new RevIMU(ahwMap);
        imu.init();
    }
}  // end of HWProfile Class