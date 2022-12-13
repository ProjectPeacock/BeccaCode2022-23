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

    //claw positions
    public final double CLAW_OPEN =0.3;
    public final double CLAW_CLOSE =0.55;

    //drive constants
    public final double MAX_DRIVE_POWER =1;
    public final double TURN_MULTIPLIER = 0.5;
    public final double STRAFE_FACTOR = 0.75;

    //anti-tip constants
    public final double ANTI_TIP_ADJ=0.3;
    public final double ANTI_TIP_TOL=10;
    public final int ANTI_TIP_AXIS=1;

    final public double DRIVE_TICKS_PER_INCH = 23.7;
    final public double USD_COUNTS_PER_INCH = 23.7;

    //lift constants
    final public double LIFT_POS_COEF = 0.05;
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
    public Motor.Encoder liftEncoder = null;


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
        //motorLF.setInverted(true);
        motorLF.resetEncoder();

        motorLR = new MotorEx(ahwMap, "motorLR", Motor.GoBILDA.RPM_1150);
        motorLR.setRunMode(Motor.RunMode.RawPower);
        motorLR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //motorLR.setInverted(true);
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
        motorLiftFront = new MotorEx(ahwMap, "motorLiftFront", Motor.GoBILDA.RPM_223);
        motorLiftRear = new MotorEx(ahwMap, "motorLiftFront", Motor.GoBILDA.RPM_223);

        //establish motorgroup for lift and set mode
        winchMotors = new MotorGroup (motorLiftFront, motorLiftRear);
        winchMotors.setRunMode(Motor.RunMode.RawPower);
        winchMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //init servos
        servoGrabber = new SimpleServo(ahwMap,"servoGrabber",0.3,0.55, AngleUnit.RADIANS);

        //init imu
        imu = new RevIMU(ahwMap);
        imu.init();
    }
    public double gyro360SelectAxis(int angle, double originAngle){
        double current = imu.getAngles()[angle];
        double rotationalAngle;

        if (originAngle > 0){
            if ((current >= 0) && (current <= 180)) {
                rotationalAngle = current;
            } else {
                rotationalAngle = 180 + (180 + current);
            }// end if(currentZ <=0) - else
        } else {
            if ((current <= 0) && (current >= -180)) {
                rotationalAngle = current;
            } else {
                rotationalAngle = -180 - (180 - current);
            }   // end if(currentZ <=0) - else
        }   // end if(targetAngle >0)-else

        return rotationalAngle;
    }// end of method gyro360
}  // end of HWProfile Class