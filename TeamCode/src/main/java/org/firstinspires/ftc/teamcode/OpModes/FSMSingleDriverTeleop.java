package org.firstinspires.ftc.teamcode.OpModes;


import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

import java.util.List;

@TeleOp(name = "FSM Single Driver Teleop Mode", group = "Competition")

public class FSMSingleDriverTeleop extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();
    public enum LiftState {
        LIFT_START,
        LIFT_RUNNING,
    };
    FSMLiftTest.LiftState liftState= FSMLiftTest.LiftState.LIFT_START;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        ButtonReader aReader = new ButtonReader(gp1, GamepadKeys.Button.A);
        ButtonReader bReader = new ButtonReader(gp1, GamepadKeys.Button.RIGHT_BUMPER);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        boolean clawToggle=false, clawReady=false, slowToggle=false, slowReady=false;
        boolean antiTip=true;
        double forwardPower=0, strafePower=0, liftPower=.5;
        int liftPos=0;

        waitForStart();
        double startTilt=robot.imu.getAngles()[robot.ANTI_TIP_AXIS], currentTilt=0, tip=0;
/*
        robot.winchMotors.resetEncoder();
*/
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        while (opModeIsActive()) {
            switch(liftState) {
                case LIFT_START:
                    if (gp1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                        robot.winch.setTargetPosition(robot.LIFT_BOTTOM);
                        liftState = FSMLiftTest.LiftState.LIFT_RUNNING;
                    } else if (gp1.isDown(GamepadKeys.Button.B)) {
                        robot.winch.setTargetPosition(robot.LIFT_LOW);
                        liftState = FSMLiftTest.LiftState.LIFT_RUNNING;
                    } else if (gp1.isDown(GamepadKeys.Button.X)) {
                        robot.winch.setTargetPosition(robot.LIFT_MID);
                        liftState = FSMLiftTest.LiftState.LIFT_RUNNING;
                    } else if (gp1.isDown(GamepadKeys.Button.Y)) {
                        robot.winch.setTargetPosition(robot.LIFT_HIGH);
                        liftState = FSMLiftTest.LiftState.LIFT_RUNNING;
                    }
                    break;

                case LIFT_RUNNING:
                    while (!robot.winch.atTargetPosition()) {
                        robot.winch.set(robot.LIFT_POW);
                    }
                    robot.winch.stopMotor();
                    liftState= FSMLiftTest.LiftState.LIFT_START;
                    break;

                default:
                    liftState = FSMLiftTest.LiftState.LIFT_START;
            }
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
            if(bReader.isDown()&&slowReady){
                slowToggle=!slowToggle;
            }

            if(!bReader.isDown()){
                slowReady=true;
            }else{
                slowReady=false;
            }
            if (slowToggle) {
                forwardPower*=0.5;
                strafePower*=0.5;
            } else {
                forwardPower*=1;
                strafePower*=1;
            }


            //mecanum drive setups
            if(robot.fieldCentric){
                //field centric setup
                robot.mecanum.driveFieldCentric(strafePower,forwardPower,-gp1.getRightX()*robot.TURN_MULTIPLIER,robot.imu.getRotation2d().getDegrees()+180, true);
            }else{
                //robot centric setup
                robot.mecanum.driveRobotCentric(strafePower,forwardPower,-gp1.getRightX()*robot.TURN_MULTIPLIER, true);
            }


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

            /*rawPower lift control
            if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1) {
                liftPower=-gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            }else if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .1){
                liftPower=gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            }else{
                liftPower=0;
            }
             */



            // Provide user feedback
            //telemetry.addData("lift position = ", robot.liftEncoder.getPosition());
            telemetry.addData("Lift Position = ", liftPos);
            telemetry.addData("Lift power = ",liftPower);
            telemetry.addData("Claw open = ", clawToggle);
            telemetry.addData("Current tip = ",tip);
            telemetry.addData("IMU Angles X = ", robot.imu.getAngles()[0]);
            telemetry.addData("IMU Angles Y = ", robot.imu.getAngles()[1]);
            telemetry.addData("IMU Angles Z = ", robot.imu.getAngles()[2]);
            telemetry.addData("Inches traveled forward/backward ", robot.forwardBackwardOdo.getPosition()/(2*Math.PI*0.7480314960629921)); //taken from odometry, long number is wheel radius in inches
            telemetry.addData("Inches traveled side/side ", robot.sideSideOdo.getPosition()/(2*Math.PI*0.7480314960629921));
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class