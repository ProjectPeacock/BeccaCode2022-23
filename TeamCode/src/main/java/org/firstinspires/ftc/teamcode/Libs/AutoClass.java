package org.firstinspires.ftc.teamcode.Libs;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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


    public void liftPosition(int position, double power){
        robot.winch.setTargetPosition(position);
        robot.winch.set(power);
    }

    /**
     * Method: liftReset
     *  -   reset the lift to starting position
     */
    public void liftReset(){
        robot.winch.setTargetPosition(robot.LIFT_BOTTOM);
        robot.winch.set(robot.LIFT_POW);
    }

    /**
     * Method: openClaw
     *  -   open the claw
     */

    public void openClaw(){
        robot.servoGrabber.setPosition(robot.CLAW_OPEN);
    }

    public void closeClaw(){
        robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
    }

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

    public double getZAngle(){
        return (-robot.imu.getRotation2d().getDegrees());
    }   // close getZAngle method


}   // close the driveMecanum class