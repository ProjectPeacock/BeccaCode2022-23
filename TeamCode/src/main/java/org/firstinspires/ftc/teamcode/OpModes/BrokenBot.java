package org.firstinspires.ftc.teamcode.OpModes;


import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;

import java.util.List;

@TeleOp(name = "Broken Bot", group = "Competition")

public class BrokenBot extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode() {
        boolean fieldCentric = false;
        boolean liftToPosition = true;
        LinearOpMode opMode = this;

        robot.init(hardwareMap);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        ButtonReader aReader = new ButtonReader(gp1, GamepadKeys.Button.A);

        if(liftToPosition){
            robot.winchMotors.setRunMode(Motor.RunMode.PositionControl);
            robot.winchMotors.setPositionCoefficient(robot.LIFT_POS_COEF);
            robot.winchMotors.setPositionTolerance(10);
            robot.winchMotors.set(1);
        }

        DriveClass drive = new DriveClass(robot, opMode);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        double liftPower=0;
        boolean clawToggle=false, clawReady=false;

        waitForStart();

        while (opModeIsActive()) {
            if(fieldCentric){
                robot.mecanum.driveFieldCentric(gp1.getLeftX(),gp1.getLeftY(),-gp1.getRightX()*robot.TURN_MULTIPLIER,robot.imu.getRotation2d().getDegrees()+180, false);
            }else{
                robot.mecanum.driveRobotCentric(gp1.getLeftX(),gp1.getLeftY(),-gp1.getRightX()*robot.TURN_MULTIPLIER, false);
            }

            if(liftToPosition){

            }else {
                if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
                    liftPower = gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
                } else if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
                    liftPower = -gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                } else {
                    liftPower = 0;
                }
            }

            if(aReader.isDown()&&clawReady){
                clawToggle=!clawToggle;
            }
            if(!aReader.isDown()){
                clawReady=true;
            }else{
                clawReady=false;
            }
            if (clawToggle) {
                robot.servoGrabber.setPosition(robot.CLAW_OPEN);
            } else {
                robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
            }
            robot.winchMotors.set(liftPower);


            if(gp1.isDown(GamepadKeys.Button.DPAD_UP)) {
                robot.motorLF.set(1);
            } else if (gp1.isDown(GamepadKeys.Button.DPAD_DOWN)){
                robot.motorLR.set(1);
            } else if (gp1.isDown(GamepadKeys.Button.DPAD_LEFT)) {
                robot.motorRF.set(1);
            } else if (gp1.isDown(GamepadKeys.Button.DPAD_RIGHT)){
                robot.motorRR.set(1);
            }


            /*
             * #############################################################
             * #################### LIFT CONTROL ###########################
             * #############################################################
             */

            if(gp1.isDown(GamepadKeys.Button.A)){
                robot.winchMotors.setTargetPosition(0);
            }

            if(gp1.isDown(GamepadKeys.Button.B)){
                robot.winchMotors.setTargetPosition(robot.JUNCTION_LOWER);
            }

            if(gp1.isDown(GamepadKeys.Button.X)){
                robot.winchMotors.setTargetPosition(robot.JUNCTION_MID);
            }

            if(gp1.isDown(GamepadKeys.Button.Y)){
                robot.winchMotors.setTargetPosition(robot.JUNCTION_HIGH);
            }


            // Provide user feedback
            telemetry.addData("A:", "Lift Bottom");
            telemetry.addData("B:", "Lift Low");
            telemetry.addData("X:", "Lift Mid");
            telemetry.addData("Y:", "Lift High");
            //telemetry.addData("lift position:", robot.winchMotors.getCurrentPosition());
            telemetry.addData("IMU Angle X = ", robot.imu.getAngles()[0]);
            telemetry.addData("IMU Angle Y = ", robot.imu.getAngles()[1]);
            telemetry.addData("IMU Angle Z = ", robot.imu.getAngles()[2]);
            telemetry.addData("Left Stick X = ", gp1.getLeftX());
            telemetry.addData("Left Stick Y = ", gp1.getLeftY());
            telemetry.addData("Right Stick X = ", gp1.getRightX());
            telemetry.addData("Right Stick Y = ", gp1.getRightY());
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class