package org.firstinspires.ftc.teamcode.OpModes;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
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
        double power = robot.MAX_DRIVE_POWER;
        robot.init(hardwareMap);

        GamepadEx gp1 = new GamepadEx(gamepad1);

        if(robot.fieldCentric){
            robot.mecanum.driveFieldCentric(gp1.getLeftX(),gp1.getLeftY(),gp1.getRightX(),robot.imu.getRotation2d().getDegrees(), false);
        }else{
            robot.mecanum.driveRobotCentric(gp1.getLeftX(),gp1.getLeftY(),gp1.getRightX(), false);
        }

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        double liftPower=0;

        ToggleButtonReader aReader = new ToggleButtonReader(gp1, GamepadKeys.Button.A);

        waitForStart();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while (opModeIsActive()) {
            if(gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.25){
                liftPower=-gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            }else if(gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.25){
                liftPower=gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            }else{
                liftPower=0;
            }

            if (aReader.getState()) {
                robot.servoGrabber.setPosition(robot.CLAW_OPEN);
            } else {
                robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
            }
            robot.motorLiftFront.set(liftPower);
            robot.motorLiftRear.set(liftPower);


            // Provide user feedback
            telemetry.addData("lift 1 position = ", robot.motorLiftFront.getCurrentPosition());
            telemetry.addData("lift 2 position = ", robot.motorLiftRear.getCurrentPosition());
            telemetry.addData("IMU Angles (X, Y, Z) = ", robot.imu.getAngles());
            telemetry.addData("dpad_up = ", gamepad1.dpad_up);
            telemetry.addData("dpad_down = ", gamepad1.dpad_down);
            telemetry.addData("dpad_left = ", gamepad1.dpad_left);
            telemetry.addData("dpad_right = ", gamepad1.dpad_right);
            telemetry.addData("Left Stick X = ", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y = ", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X = ", gamepad1.right_stick_x);
            telemetry.addData("Right Stick Y = ", gamepad1.right_stick_y);
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class