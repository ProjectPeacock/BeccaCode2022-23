package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
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
<<<<<<< Updated upstream
    final public int MAX_LIFT_VALUE = 5000;
    final public int MIN_LIFT_VALUE = 0;
    final public int JUNCTION_LOWER = 2000;
    final public int JUNCTION_MID = 4000;
    final public int JUNCTION_HIGH = 5000;
=======
    final public int MAX_LIFT_VALUE = 1275;
    final public int LIFT_RESET = 0;
    final public int LIFT_JUNCTION_LOWER = 400;
    final public int LIFT_JUNCTION_MID = 850;
    final public int LIFT_JUNCTION_HIGH = 1200;
    final public int LIFT_CONE5 = 800;
    final public int LIFT_CONE4 = 600;
    final public int LIFT_CONE3 = 400;
    final public int LIFT_CONE2 = 200;
    final public double LIFT_UP_POWER = 1;
    final public double LIFT_DOWN_POWER = 0.5;

    final public double PARK_TIME = 27;     // sets the time for when the robot needs to park in auto
>>>>>>> Stashed changes

    final public double MIN_PIDROTATE_POWER = 0.2;

    /* Public OpMode members. */
    public MotorEx motorLF = null;
    public MotorEx motorLR = null;
    public MotorEx motorRF = null;
    public MotorEx motorRR = null;
<<<<<<< Updated upstream
    public MotorEx motorLiftFront = null;
    public MotorEx motorLiftRear = null;
    public RevIMU imu = null;
    public ServoEx servoGrabber = null;
    public MecanumDrive mecanum = null;
    public MotorGroup winchMotors = null;
    public Motor.Encoder liftEncoder = null;
    public Motor autoLight = null;
=======
    public DcMotorEx motorLiftFront = null;
    public DcMotorEx motorLiftRear = null;
    public RevIMU imu = null;
    public ServoEx servoGrabber = null;
    public MecanumDrive mecanum = null;
    public DcMotorEx autoLight = null;
>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
        motorLF.setRunMode(Motor.RunMode.RawPower);
        motorLF.setInverted(true);
=======
        motorLF.setInverted(true);
        motorLF.setRunMode(Motor.RunMode.RawPower);
>>>>>>> Stashed changes
        motorLF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLF.resetEncoder();

        motorLR = new MotorEx(ahwMap, "motorLR", Motor.GoBILDA.RPM_1150);
<<<<<<< Updated upstream
        motorLR.setRunMode(Motor.RunMode.RawPower);
        motorLR.setInverted(true);
=======
        motorLR.setInverted(true);
        motorLR.setRunMode(Motor.RunMode.RawPower);
>>>>>>> Stashed changes
        motorLR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLR.resetEncoder();

        motorRF = new MotorEx(ahwMap, "motorRF", Motor.GoBILDA.RPM_1150);
<<<<<<< Updated upstream
=======
        motorRF.setInverted(true);
>>>>>>> Stashed changes
        motorRF.setRunMode(Motor.RunMode.RawPower);
        motorRF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRF.resetEncoder();

        motorRR = new MotorEx(ahwMap, "motorRR", Motor.GoBILDA.RPM_1150);
<<<<<<< Updated upstream
=======
        motorRR.setInverted(true);
>>>>>>> Stashed changes
        motorRR.setRunMode(Motor.RunMode.RawPower);
        motorRR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRR.resetEncoder();

<<<<<<< Updated upstream
        autoLight = new Motor(ahwMap,"autoLight", Motor.GoBILDA.BARE);
        autoLight.setRunMode(Motor.RunMode.RawPower);

=======
>>>>>>> Stashed changes
        //drivebase init
        mecanum = new MecanumDrive(motorLF, motorRF, motorLR, motorRR);

        //lift motors init
<<<<<<< Updated upstream
        MotorEx motorLiftFront = new MotorEx(ahwMap, "motorLiftFront", Motor.GoBILDA.RPM_1150);
        motorLiftFront.setRunMode(Motor.RunMode.RawPower);
        motorLiftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        MotorEx motorLiftRear = new MotorEx(ahwMap, "motorLiftRear", Motor.GoBILDA.RPM_1150);
        motorLiftRear.setRunMode(Motor.RunMode.RawPower);
        motorLiftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //establish motorgroup for lift and set mode
        winchMotors = new MotorGroup (motorLiftFront, motorLiftRear);
        winchMotors.setRunMode(Motor.RunMode.RawPower);
        winchMotors.setPositionCoefficient(0.05);
        winchMotors.setPositionTolerance(10);
        winchMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
=======
        motorLiftFront = hwMap.get(DcMotorEx.class, "motorLiftFront");
        motorLiftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLiftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftFront.setTargetPosition(0);
        motorLiftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftFront.setPower(0);

        motorLiftRear = hwMap.get(DcMotorEx.class, "motorLiftRear");
        motorLiftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLiftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftRear.setTargetPosition(0);
        motorLiftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftRear.setPower(0);

        autoLight = hwMap.get(DcMotorEx.class, "autoLight");
        autoLight.setPower(0);
>>>>>>> Stashed changes

        //init servos
        servoGrabber = new SimpleServo(ahwMap,"servoGrabber",0.3,0.55, AngleUnit.RADIANS);

        //init imu
        imu = new RevIMU(ahwMap);
        imu.init();
    }
}  // end of HWProfile Class