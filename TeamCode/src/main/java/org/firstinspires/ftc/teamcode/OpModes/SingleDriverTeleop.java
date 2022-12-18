package org.firstinspires.ftc.teamcode.OpModes;


import android.widget.Button;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

import java.util.List;

@TeleOp(name = "Single Driver Teleop Mode", group = "Competition")

public class SingleDriverTeleop extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        ButtonReader aReader = new ButtonReader(gp1, GamepadKeys.Button.A);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        double liftPower=0;
        boolean clawToggle=false, clawReady=false;
        boolean antiTip=true;
        double forwardPower=0, strafePower=0;

        waitForStart();
        double startTilt=robot.imu.getAngles()[robot.ANTI_TIP_AXIS], currentTilt=0, tip=0;

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while (opModeIsActive()) {
            forwardPower=gp1.getLeftY();
            strafePower=gp1.getLeftX();

            //anti-tip "algorithm"
            if(antiTip){
                currentTilt=robot.imu.getAngles()[robot.ANTI_TIP_AXIS];
                tip = Math.abs(currentTilt-startTilt);
                //if robot is tipped more than tolerance, multiply drive power by adjustment
                if(tip>robot.ANTI_TIP_TOL*2){
                    forwardPower*=-1;
                }else if(tip>robot.ANTI_TIP_TOL){
                    forwardPower*=robot.ANTI_TIP_ADJ;
                }
            }

            //mecanum drive setups
            if(robot.fieldCentric){
                //field centric setup
                robot.mecanum.driveFieldCentric(strafePower,forwardPower,-gp1.getRightX()*robot.TURN_MULTIPLIER,robot.imu.getRotation2d().getDegrees()+180, false);
            }else{
                //robot centric setup
                robot.mecanum.driveRobotCentric(strafePower,forwardPower,-gp1.getRightX()*robot.TURN_MULTIPLIER, false);
            }

            //lift power (take analog from triggers, apply to variable, variable gets applied to motors
            if(gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.1){
                liftPower=gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            }else if(gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.1){
                liftPower=-gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            }else{
                liftPower=0;
            }
            robot.winchMotors.set(liftPower);

            //claw control
            if(aReader.isDown()&&clawReady){
                clawToggle=!clawToggle;
            }
            //forces claw to only open or close if button is pressed once, not held
            if(!aReader.isDown()){
                clawReady=true;
            }else{
                clawReady=false;
            }
            //apply value to claw
            if (clawToggle) {
                robot.servoGrabber.setPosition(robot.CLAW_OPEN);
            } else {
                robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
            }

            // Provide user feedback
            //telemetry.addData("lift position = ", robot.liftEncoder.getPosition());
            telemetry.addData("Lift power = ",liftPower);
            telemetry.addData("Claw open = ", clawToggle);
            telemetry.addData("Current tip = ",tip);
            telemetry.addData("IMU Angles X = ", robot.imu.getAngles()[0]);
            telemetry.addData("IMU Angles Y = ", robot.imu.getAngles()[1]);
            telemetry.addData("IMU Angles Z = ", robot.imu.getAngles()[2]);
            telemetry.addData("dpad_up = ", gp1.getButton(GamepadKeys.Button.DPAD_UP));
            telemetry.addData("dpad_down = ", gp1.getButton(GamepadKeys.Button.DPAD_DOWN));
            telemetry.addData("dpad_left = ", gp1.getButton(GamepadKeys.Button.DPAD_LEFT));
            telemetry.addData("dpad_right = ", gp1.getButton(GamepadKeys.Button.DPAD_RIGHT));
            telemetry.addData("Left Stick X = ", gp1.getLeftX());
            telemetry.addData("Left Stick Y = ", gp1.getLeftY());
            telemetry.addData("Right Stick X = ", gp1.getRightX());
            telemetry.addData("Right Stick Y = ", gp1.getRightY());
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class