package org.firstinspires.ftc.teamcode.Libs;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

public class AutoClass {

    private HWProfile robot;
    public double RF, LF, LR, RR;
    public LinearOpMode opMode;
    ElapsedTime runTime = new ElapsedTime();


    /*
     * Constructor method
     */
    public AutoClass(HWProfile myRobot, LinearOpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;

    }   // close AutoClass constructor Method


    /**
     * Method: liftReset
     *  -   reset the lift to starting position
     */

    //method for moving lift to score and retract
    //pos corresponds to bottom, low, mid, high
    //0==bottom
    //1==low
    //2==mid
    //3==high
    public void moveLiftScore(int pos){
        robot.winch.set(0);
        if(pos==0){
            robot.winch.setTargetPosition(robot.LIFT_BOTTOM);
        }else if(pos==1){
            robot.winch.setTargetPosition(robot.LIFT_LOW);
        }else if(pos==2){
            robot.winch.setTargetPosition(robot.LIFT_MID);
        }else if(pos==3){
            robot.winch.setTargetPosition(robot.LIFT_HIGH);
        }
        while(!robot.winch.atTargetPosition()){
            robot.winch.set(1);
        }
        robot.winch.stopMotor();
    }

    //method for moving lift to retrieve cones
    //cycle corresponds to what number cycle the robot is on
    public void moveLiftGrab(int cycle){
        robot.winch.set(0);
        if(cycle==1){
            robot.winch.setTargetPosition(robot.LIFT_BOTTOM);
        }else if(cycle==2){
            robot.winch.setTargetPosition(robot.LIFT_LOW);
        }else if(cycle==3){
            robot.winch.setTargetPosition(robot.LIFT_MID);
        }else if(cycle==4){
            robot.winch.setTargetPosition(robot.LIFT_HIGH);
        }
        else if(cycle==5){
            robot.winch.setTargetPosition(robot.LIFT_HIGH);
        }

        while(!robot.winch.atTargetPosition()){
            robot.winch.set(1);
        }
        robot.winch.stopMotor();
    }

    //claw control methods
    public void openClaw(){
        robot.servoGrabber.setPosition(robot.CLAW_OPEN);
    }
    public void closeClaw(){
        robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
    }

    //method to get robot heading (used to store final heading for field-centric offset)
    private double gyro360(double targetAngle){
        double currentZ = getZAngle();
        double rotationalAngle;

        if (targetAngle > 0){
            if ((currentZ >= 0) && (currentZ <= 180)) {
                rotationalAngle = currentZ;
            } else {
                rotationalAngle = 180 + (180 + currentZ);
            }// end if(currentZ <=0) - else
        } else {
            if ((currentZ <= 0) && (currentZ >= -180)) {
                rotationalAngle = currentZ;
            } else {
                rotationalAngle = -180 - (180 - currentZ);
            }   // end if(currentZ <=0) - else
        }   // end if(targetAngle >0)-else

        return rotationalAngle;
    }   // end method gyro360

    //support method for gyro360 method
    private double getZAngle(){
        return (-robot.imu.getRotation2d().getDegrees());
    }   // close getZAngle method



}   // close the AutoClass class