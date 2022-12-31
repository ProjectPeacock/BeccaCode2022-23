package org.firstinspires.ftc.teamcode.Libs;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
<<<<<<< Updated upstream
import com.qualcomm.robotcore.util.Range;
=======
>>>>>>> Stashed changes

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


<<<<<<< Updated upstream
    public void liftPosition(int position){
        robot.motorLiftFront.setTargetPosition(position);
        robot.motorLiftFront.set(1);
=======
    public void liftPosition(int position, double power){
        robot.motorLiftFront.setTargetPosition(position);
        robot.motorLiftRear.setTargetPosition(position);
        robot.motorLiftFront.setPower(power);
        robot.motorLiftFront.setPower(power);
>>>>>>> Stashed changes
    }

    /**
     * Method: liftReset
     *  -   reset the lift to starting position
     */
    public void liftReset(){
<<<<<<< Updated upstream
        robot.motorLiftFront.setTargetPosition(0);
        robot.motorLiftFront.set(1);
    }

    /**
     * Method: liftLowerJunction
     *  -   raise the lift to the lower junction level
     */
    public void liftLowerJunction(){
        robot.motorLiftFront.setTargetPosition(robot.JUNCTION_LOWER);
        robot.motorLiftFront.set(1);
    }

    /**
     * Method: liftMidJunction
     *  -   raise the lift to the mid junction level
     */
    public void liftMidJunction(){
        robot.motorLiftFront.setTargetPosition(robot.JUNCTION_MID);
        robot.motorLiftFront.set(1);
    }

    /**
     * Method: liftHighJunction
     *  -   raise the lift to the lower junction level
     */
    public void liftHighJunction(){
        robot.motorLiftFront.setTargetPosition(robot.JUNCTION_HIGH);
        robot.motorLiftFront.set(1);
=======
        robot.motorLiftFront.setTargetPosition(robot.LIFT_RESET);
        robot.motorLiftRear.setTargetPosition(robot.LIFT_RESET);
        robot.motorLiftFront.setPower(robot.LIFT_DOWN_POWER);
        robot.motorLiftFront.setPower(robot.LIFT_DOWN_POWER);
>>>>>>> Stashed changes
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