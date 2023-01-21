package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "FSM Lift Test", group = "Competition")
//@Disabled
public class FSMLiftTest extends OpMode {
    public enum LiftState {
        LIFT_START,
        LIFT_RUNNING,
    };
    //default to starting case
    LiftState liftState=LiftState.LIFT_START;

    public MotorEx motorLiftFront = null;
    public MotorEx motorLiftRear = null;
    public MotorGroup winch = null;
    FtcDashboard dashboard;
    //public MotorGroup winch = null;
    public static int LIFT_BOTTOM=0;
    public static int LIFT_LOW=100;
    public static int LIFT_MID=500;
    public static int LIFT_HIGH=800;
    public static double LIFT_KP=0.0225;
    public static int LIFT_TOL=10;

    public void init(){
        motorLiftFront = new MotorEx(hardwareMap,"motorLiftFront", Motor.GoBILDA.RPM_1150);
        motorLiftRear = new MotorEx(hardwareMap,"motorLiftRear", Motor.GoBILDA.RPM_1150);

        winch = new MotorGroup(motorLiftFront,motorLiftRear);
        winch.setRunMode(Motor.RunMode.PositionControl);
        winch.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        winch.setPositionCoefficient(LIFT_KP);
        winch.setPositionTolerance(LIFT_TOL);
        winch.resetEncoder();
        dashboard = FtcDashboard.getInstance();
        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();
    }

    public void loop() {
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        dashTelemetry.put("01 - Winch Motor Positions", winch.getPositions());
        dashTelemetry.put("02 - GP1.Button.A = ", "RESET LIFT");
        dashTelemetry.put("03 - GP1.Button.B = ", "LIFT LOW JUNCTION");
        dashTelemetry.put("04 - GP1.Button.X = ", "LIFT MID JUNCTION");
        dashTelemetry.put("05 - GP1.Button.Y = ", "LIFT HIGH JUNCTION");
        dashboard.sendTelemetryPacket(dashTelemetry);

        winch.setPositionTolerance(LIFT_TOL);
        winch.setPositionCoefficient(LIFT_KP);
        winch.set(1);

        switch(liftState) {
            case LIFT_START:
                if (gamepad1.a) {
                    winch.setTargetPosition(LIFT_BOTTOM);
                    liftState = LiftState.LIFT_RUNNING;
                } else if (gamepad1.b) {
                    winch.setTargetPosition(LIFT_LOW);
                    liftState = LiftState.LIFT_RUNNING;
                } else if (gamepad1.x) {
                    winch.setTargetPosition(LIFT_MID);
                    liftState = LiftState.LIFT_RUNNING;
                } else if (gamepad1.y) {
                    winch.setTargetPosition(LIFT_HIGH);
                    liftState = LiftState.LIFT_RUNNING;
                }
                break;

            case LIFT_RUNNING:
                while (!winch.atTargetPosition()) {
                    winch.set(1);
                }
                winch.stopMotor();
                liftState=LiftState.LIFT_START;
                break;

            default:
                liftState = LiftState.LIFT_START;
        }
        if (gamepad1.a && liftState != LiftState.LIFT_START) {
            liftState = LiftState.LIFT_START;
        }

        // Provide user feedback
        dashTelemetry.put("01 - Winch Motor Positions", winch.getPositions());
        dashTelemetry.put("02 - GP1.Button.A = ", "RESET LIFT");
        dashTelemetry.put("03 - GP1.Button.B = ", "LIFT LOW JUNCTION");
        dashTelemetry.put("04 - GP1.Button.X = ", "LIFT MID JUNCTION");
        dashTelemetry.put("05 - GP1.Button.Y = ", "LIFT HIGH JUNCTION");
        dashboard.sendTelemetryPacket(dashTelemetry);

        telemetry.addData("Winch Motor Positions: ",winch.getPositions());
        telemetry.addData("Winch KP: ",winch.getPositionCoefficient());
        telemetry.addData("GP1.Button.A: ","RESET LIFT");
        telemetry.addData("GP1.Button.B: ","LIFT LOW JUNCTION");
        telemetry.addData("GP1.Button.X: ","LIFT MID JUNCTION");
        telemetry.addData("GP1.Button.Y: ","LIFT HIGH JUNCTION");
        telemetry.update();
    }   // end of while(opModeIsActive)
}   // end of runOpMode()