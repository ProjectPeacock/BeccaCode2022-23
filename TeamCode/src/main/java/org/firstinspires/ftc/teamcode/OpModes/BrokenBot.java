package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;

@TeleOp(name = "Broken Bot", group = "Competition")

public class BrokenBot extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode() {
        boolean fieldCentric = false;
        int liftPosition = 0;
        LinearOpMode opMode = this;

        ElapsedTime currentTime = new ElapsedTime();
        double buttonPress = currentTime.time();

        robot.init(hardwareMap);

        DriveClass drive = new DriveClass(robot, opMode);


        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        boolean clawOpen = true;

        waitForStart();

        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/


            if(gamepad1.dpad_up) {
                robot.motorLF.set(1);
            } else if (gamepad1.dpad_down) {
                robot.motorLR.set(1);
            } else if (gamepad1.dpad_left) {
                robot.motorRF.set(1);
            } else if (gamepad1.dpad_right){
                robot.motorRR.set(1);
            }


            /*
             * #############################################################
             * #################### LIFT CONTROL ###########################
             * #############################################################
             */

            // limit the values of liftPosition => This shouldn't be necessary if logic above works
            Range.clip(liftPosition, robot.MIN_LIFT_VALUE, robot.MAX_LIFT_VALUE);

            if(gamepad2.a){
                drive.liftRearTest(0);
            }

            if(gamepad2.b){
                drive.liftRearTest(robot.JUNCTION_LOWER);
            }

            if(gamepad2.x){
                drive.liftFrontTest(0);
            }

            if(gamepad2.y){
                drive.liftFrontTest(robot.JUNCTION_LOWER);
            }

            if(gamepad1.a&&(currentTime.time() - buttonPress) > robot.BUTTON_TIMEOUT){
                clawOpen=!clawOpen;
                buttonPress = currentTime.time();
            }
/*
            if (clawOpen) {
                robot.servoGrabber.setPosition(robot.CLAW_OPEN);
            } else {
                robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
            */


            // Provide user feedback
            telemetry.addData("lift position:", robot.winchMotors.getCurrentPosition());
            telemetry.addData("MotorLR:", robot.motorLR.getCurrentPosition());
            telemetry.addData("MotorLF:", robot.motorLF.getCurrentPosition());
            telemetry.addData("MotorRF:", robot.motorRF.getCurrentPosition());
            telemetry.addData("MotorRR:", robot.motorRR.getCurrentPosition());
            telemetry.addData("IMU Angles (X, Y, Z) = ", robot.imu.getAngles());
            telemetry.addData("Left Stick X = ", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y = ", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X = ", gamepad1.right_stick_x);
            telemetry.addData("Right Stick Y = ", gamepad1.right_stick_y);
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class