package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
@Config
@TeleOp(name = "Lift Test", group = "Competition")
//@Disabled
public class LiftTest extends LinearOpMode {
    public DcMotorEx motorLiftFront = null;
    public DcMotorEx motorLiftRear = null;
    FtcDashboard dashboard;
    //public MotorGroup winch = null;
    public static int LIFT_BOTTOM=0;
    public static int LIFT_LOW=100;
    public static int LIFT_MID=500;
    public static int LIFT_HIGH=800;
    public static double LIFT_POW_DOWN=0.3;
    public static double LIFT_POW_UP=1;


    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        double liftPower=0;
        int liftPos=0;
        int lastPosition=0;

        //lift motor init
        //motorLiftFront = new MotorEx(hardwareMap, "motorLiftFront", Motor.GoBILDA.RPM_1150);
        //motorLiftRear = new MotorEx(hardwareMap, "motorLiftRear", Motor.GoBILDA.RPM_1150);
        motorLiftFront = hardwareMap.get(DcMotorEx.class, "motorLiftFront");
        motorLiftRear = hardwareMap.get(DcMotorEx.class, "motorLiftRear");
        motorLiftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /*
        winch = new MotorGroup(motorLiftFront,motorLiftRear);
        winch.setRunMode(Motor.RunMode.PositionControl);
        winch.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        winch.setPositionCoefficient(LIFT_KP);
        winch.setPositionTolerance(LIFT_TOL);
        winch.resetEncoder();
         */

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        dashTelemetry.put("01 - Lift Target Position ", liftPos);
        dashTelemetry.put("02 - Lift Actual Position", motorLiftFront.getCurrentPosition());
        //dashTelemetry.put("02 - Lift Actual Position", winch.getPositions());
        dashTelemetry.put("03 - Lift Power = ", liftPower);
        dashTelemetry.put("04 - GP1.Button.A = ", "RESET LIFT");
        dashTelemetry.put("05 - GP1.Button.B = ", "LIFT LOW JUNCTION");
        dashTelemetry.put("06 - GP1.Button.X = ", "LIFT MID JUNCTION");
        dashTelemetry.put("07 - GP1.Button.Y = ", "LIFT HIGH JUNCTION");
        dashboard.sendTelemetryPacket(dashTelemetry);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a){
                liftPos=LIFT_BOTTOM;
            }

            if(gamepad1.b){
                liftPos=LIFT_LOW;
            }

            if(gamepad1.x){
                liftPos=LIFT_MID;
            }

            if(gamepad1.y){
                liftPos=LIFT_HIGH;
            }

            /*
            winch.setTargetPosition(liftPos);
            while(!winch.atTargetPosition()) {
//                winch.setTargetPosition(liftPos);
                if (liftPos > lastPosition) {
                    winch.set(LIFT_POW_UP);
                } else {
                    winch.set(LIFT_POW_DOWN);
                }
            }
             */
            motorLiftFront.setTargetPosition(liftPos);
            motorLiftRear.setTargetPosition(liftPos);
            if(liftPos>motorLiftFront.getCurrentPosition()){
                liftPower=LIFT_POW_UP;
            }else{
                liftPower=LIFT_POW_DOWN;
            }
            motorLiftFront.setPower(1);
            motorLiftRear.setPower(1);

            // Provide user feedback
            dashTelemetry.put("01 - Lift Target Position ", liftPos);
            dashTelemetry.put("02 - Lift Front Position", motorLiftFront.getCurrentPosition());
            dashTelemetry.put("02 - Lift Rear Position", motorLiftRear.getCurrentPosition());
            dashTelemetry.put("03 - Lift Front Power = ", motorLiftFront.getPower());
            dashTelemetry.put("03 - Lift Rear Power = ", motorLiftRear.getPower());
            dashTelemetry.put("04 - GP1.Button.A = ", "RESET LIFT");
            dashTelemetry.put("05 - GP1.Button.B = ", "LIFT LOW JUNCTION");
            dashTelemetry.put("06 - GP1.Button.X = ", "LIFT MID JUNCTION");
            dashTelemetry.put("07 - GP1.Button.Y = ", "LIFT HIGH JUNCTION");
            dashboard.sendTelemetryPacket(dashTelemetry);

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of LiftTest class