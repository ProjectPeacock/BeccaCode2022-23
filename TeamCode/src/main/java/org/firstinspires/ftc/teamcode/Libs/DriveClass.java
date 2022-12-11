package org.firstinspires.ftc.teamcode.Libs;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

public class DriveClass {

    private HWProfile robot;
    public double RF, LF, LR, RR;
    public LinearOpMode opMode;
    ElapsedTime runTime = new ElapsedTime();

    /*
     * Constructor method
     */
    public DriveClass(HWProfile myRobot, LinearOpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;

    }   // close DriveMecanum constructor Method

    /**
     * ###########################################################################################
     * ###########################################################################################
     * #######################      DRIVE CONTROLS      ##########################################
     * ###########################################################################################
     * ###########################################################################################
     */

    /**
     *  Method: driveDistance
     *  -   uses the encoder values to determine distance traveled.
     *  -   This method will autocorrect the position of the robot if it drifts off its position
     *  -   Note: This method uses gyro360 to measure it's angle. The reference target angle used
     *              is 0 degrees as that ensures that the reference and measured angles always
     *              provide consistent reporting comparisons.
     * @param power     - provides the power/speed that the robot should move
     * @param heading   - direction for the robot to strafe to
     * @param distance  - amount of time that the robot will move
     */
    public void driveDistance(double power, double heading, double distance) {
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;

        double theta = Math.toRadians(90 + heading);
        double lfStart = 0;
        double lrStart = 0;
        double rfStart = 0;
        double rrStart = 0;

        lfStart = robot.motorLF.getCurrentPosition();
        lrStart = robot.motorLR.getCurrentPosition();
        rfStart = robot.motorRF.getCurrentPosition();
        rrStart = robot.motorRR.getCurrentPosition();

        while(opMode.opModeIsActive() && active) {

            RF = power * (Math.sin(theta) - Math.cos(theta));
            LF = power * (Math.sin(theta) + Math.cos(theta));
            LR = power * (Math.sin(theta) - Math.cos(theta));
            RR = power * (Math.sin(theta) + Math.cos(theta));

            if (initZ > 170 || initZ < -170){
                currentZ = gyro360(0);      // always use 0 as the reference angle
            } else {
                currentZ = getZAngle();
            }
            if (currentZ != initZ){
                zCorrection = Math.abs(initZ - currentZ)/100;

                if (initZ < currentZ) {
                    RF = RF - zCorrection;
                    RR = RR - zCorrection;
                    LF = LF + zCorrection;
                    LR = LR + zCorrection;
                }
                if (initZ > currentZ) {
                    RF = RF + zCorrection;
                    RR = RR + zCorrection;
                    LF = LF - zCorrection;
                    LR = LR - zCorrection;
                }
            }   // end of if currentZ != initZ

            /*
             * Limit that value of the drive motors so that the power does not exceed 100%
             */
            RF = Range.clip(RF, -power,power);
            LF = Range.clip(LF, -power,power);
            RR = Range.clip(RR, -power,power);
            LR = Range.clip(LR, -power,power);

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RF, LF, LR, RR);

            /*
            opMode.telemetry.addData("LF Start = ", lfStart);
            opMode.telemetry.addData("Distance = ", distance);
            opMode.telemetry.addData("Heading = ", heading);
            opMode.telemetry.addData("Calculated Distance = ", calcDistance(heading, rfStart, rrStart, lfStart, lrStart));
            opMode.telemetry.update();
             */

            if(calcDistance(heading, rfStart, rrStart, lfStart, lrStart) >= distance) active = false;
            opMode.idle();

        }   // end of while loop

        motorsHalt();

    }   // close driveDistance method

    /**
     * Method: PIDRotate
     * Parameters:
     * @param targetAngle -> desire ending angle/position of the robot
     * @param targetError -> how close should the robot get to the desired angle
     */
    public void PIDRotate(double targetAngle, double targetError){
        double integral = 0;
        int iterations = 0;
        ElapsedTime timeElapsed = new ElapsedTime();
        double startTime = timeElapsed.time();
        double totalTime;
        double error;
        double Cp = 0.06;
        double Ci = 0.0003;
        double Cd = 0.0001;
        double maxSpeed = 0.5;
        double rotationSpeed;
        double derivative = 0, deltaError, lastError=0;

        // check to see how far the robot is rotating to decide which gyro sensor value to use
        error = updateError(targetAngle);

        // nested while loops are used to allow for a final check of an overshoot situation
        while ((Math.abs(error) >= targetError) && opMode.opModeIsActive()) {
            while ((Math.abs(error) >= targetError) && opMode.opModeIsActive()) {
                deltaError = lastError - error;
                rotationSpeed = ((Cp * error) + (Ci * integral) + (Cd * derivative)) * maxSpeed;

                // Clip motor speed
                rotationSpeed = Range.clip(rotationSpeed, -maxSpeed, maxSpeed);

                // bump the power up to min threshold if the robot doesn't have enough power to turn
                if ((rotationSpeed > -0.15) && (rotationSpeed < 0)) {
                    rotationSpeed = -robot.MIN_PIDROTATE_POWER;
                } else if ((rotationSpeed < 0.15) && (rotationSpeed > 0)) {
                    rotationSpeed = robot.MIN_PIDROTATE_POWER;
                }

                RF = -rotationSpeed;
                LF = rotationSpeed;
                LR = rotationSpeed;
                RR = -rotationSpeed;

                setDrivePower(RF, LF, LR, RR);

                lastError = error;
                iterations++;

                /*
                opMode.telemetry.addData("InitZ/targetAngle value  = ", targetAngle);
                opMode.telemetry.addData("Current Angle  = ", getZAngle());
                opMode.telemetry.addData("Theta/lastError Value= ", lastError);
                opMode.telemetry.addData("CurrentZ/Error Value = ", error);
                opMode.telemetry.addData("zCorrection/derivative Value = ", derivative);

                opMode.telemetry.addData("Right Front = ", RF);
                opMode.telemetry.addData("Left Front = ", LF);
                opMode.telemetry.addData("Left Rear = ", LR);
                opMode.telemetry.addData("Right Rear = ", RR);
                opMode.telemetry.update();
                 */

                // check to see how far the robot is rotating to decide which gyro sensor value to use
                error = updateError(targetAngle);

            }   // end of while Math.abs(error)
            motorsHalt();

            // Perform a final calc on the error to confirm that the robot didn't overshoot the
            // target position after the last measurement was taken.
            opMode.sleep(10);
            error = updateError(targetAngle);
        }   // close outside while loop

        // shut off the drive motors
        motorsHalt();

        /*
        totalTime = timeElapsed.time() - startTime;
        opMode.telemetry.addData("Iterations = ", iterations);
        opMode.telemetry.addData("Final Angle = ", getZAngle());
        opMode.telemetry.addData("Total Time Elapsed = ", totalTime);
        opMode.telemetry.update();
         */
    }   //end of the PIDRotate Method

    /**
     * Method driveByTime
     *  -   This method is a revised version of robotCorrect. There were some behaviors that needed
     *      to be fixed, but didn't want to risk all of the programs. This one can be renamed
     *      robotCorrect and all instances of robotCorrect2 changed to robotCorrect.
     * @param power
     * @param heading
     * @param duration
     */
    public void driveByTime(double power, double heading, double duration) {
        String action = "Initializing";
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;
        double theta = Math.toRadians(90 + heading);
        ElapsedTime runTime = new ElapsedTime();

        if(runTime.time() >= duration) active = false;

        while(opMode.opModeIsActive() && active) {
            updateValues(action, initZ, theta, currentZ, zCorrection);

            // calculate power to apply to each wheel
            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            if(runTime.time() >= duration) active = false;

            currentZ = getZAngle();
            if (currentZ != initZ){
                zCorrection = Math.abs(initZ - currentZ)/100;

                if (initZ < currentZ) {
                    RF = RF - zCorrection;
                    RR = RR - zCorrection;
                    LF = LF + zCorrection;
                    LR = LR + zCorrection;
                }
                if (initZ > currentZ) {
                    RF = RF + zCorrection;
                    RR = RR + zCorrection;
                    LF = LF - zCorrection;
                    LR = LR - zCorrection;
                }
            }   // end of if currentZ != initZ

            /*
             * Limit that value of the drive motors so that the power does not exceed 100%
             */
            RF = Range.clip(RF, -1,1);
            LF = Range.clip(LF, -1,1);
            RR = Range.clip(RR, -1,1);
            LR = Range.clip(LR, -1,1);

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RF, LF, LR, RR);

        }   // end of while loop

        motorsHalt();
    }   // close driveByTime method


    /**
     *  Method: driveSimpleDistance
     *  -   uses the encoder values to determine distance traveled.
     *  -   This method will NOT autocorrect the position of the robot if it drifts off path
     * @param power     - provides the power/speed that the robot should move
     * @param heading   - direction for the robot to strafe to
     * @param distance  - amount of time that the robot will move
     */
    public void driveSimpleDistance(double power, double heading, double distance) {
        boolean active = true;

        double theta = Math.toRadians(90 + heading);
        double lfStart = robot.motorLF.getCurrentPosition();
        double lrStart = robot.motorLR.getCurrentPosition();
        double rfStart = robot.motorRF.getCurrentPosition();
        double rrStart = robot.motorRR.getCurrentPosition();

        while(opMode.opModeIsActive() && active) {

            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RF, LF, LR, RR);

            if(calcDistance(heading, rfStart, rrStart, lfStart, lrStart) >= distance) active = false;

        }   // end of while loop

        motorsHalt();
    }   // close simpleDriveDistance method

    /**
     *  Method: setDrivePower
     * Sets power to all four drive motors
     * @param RF power for right front motor
     * @param LF power for left front motor
     * @param LR power for left rear motor
     * @param RR power for right rear motor
     */
    public void setDrivePower(double RF, double LF, double LR, double RR){
        robot.motorRF.setPower(RF);
        robot.motorLF.setPower(LF);
        robot.motorLR.setPower(LR);
        robot.motorRR.setPower(RR);
    }   // end of the setDrivePower method

    /**
     * Method: motorsHalt
     *  -   stops all drive motors
     */
    public void motorsHalt(){
        robot.motorRF.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    }   // end of motorsHalt method

    /*
     * ###########################################################################################
     * ###########################################################################################
     * #######################      ARM & LIFT CONTROLS      #####################################
     * ###########################################################################################
     * ###########################################################################################
     */

    /**
     * Method: liftPosition
     *  -   raise the lift to the desired position
     * @param position
     */
    public void liftPosition(int position){
        robot.motorLiftFront.setTargetPosition(position);
        robot.motorLiftFront.setPower((1));
    }

    /**
     * Method: liftReset
     *  -   reset the lift to starting position
     */
    public void liftReset(){
        robot.motorLiftFront.setTargetPosition(0);
        robot.motorLiftFront.setPower((1));
    }

    /**
     * Method: liftLowerJunction
     *  -   raise the lift to the lower junction level
     */
    public void liftLowerJunction(){
        robot.motorLiftFront.setTargetPosition(robot.JUNCTION_LOWER);
        robot.motorLiftFront.setPower(1);
    }

    /**
     * Method: liftMidJunction
     *  -   raise the lift to the mid junction level
     */
    public void liftMidJunction(){
        robot.motorLiftFront.setTargetPosition(robot.JUNCTION_MID);
        robot.motorLiftFront.setPower(1);
    }

    /**
     * Method: liftHighJunction
     *  -   raise the lift to the lower junction level
     */
    public void liftHighJunction(){
        robot.motorLiftFront.setTargetPosition(robot.JUNCTION_HIGH);
        robot.motorLiftFront.setPower(1);
    }

    /**
     * Method: openClaw
     *  -   open the claw
     */
    /*
    public void openClaw(){
        robot.servoGrabber.setPosition(robot.CLAW_OPEN);
    }

    /**
     * Method: closeClaw
     *  -   close the claw


    public void closeClaw(){
        robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
    }
    */



    /*
     * ###########################################################################################
     * ###########################################################################################
     * #######################      SYSTEM CONTROLS      #########################################
     * ###########################################################################################
     * ###########################################################################################
     */

    /**
     *  Method: updateError
     *  -   uses the gyro values to determine current angular position.
     *  -   This method will calculate the variance between the current robot angle and the target angle
     *  -   Note: this method is a sub method of PIDRotate
     * @param targetAngle     - angle that the robot would like to turn to
     */
    private double updateError(double targetAngle){
        double calculatedError = 0;

        if (targetAngle > 100 || targetAngle < -100) {
            calculatedError = gyro360(targetAngle) - targetAngle;
        } else {
            calculatedError = getZAngle() - targetAngle;}

        return(calculatedError);
    }

    /**
     * Method gyro360
     *  - Causes the Gyro to behave in 360 mode instead of 180 degree mode
     * @param targetAngle - Reference angle for the gyro sensor
     */
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

    /**
     * Method getRunTime
     * @return runtime
     */
    public double getRunTime(){
        return runTime.time();
    }

    /**
     * Method getZAngle()
     *  -   This method returns the gyro position of the robot.
     * @return zAngle
     */
    public double getZAngle(){
        return (-robot.imu.getAngularOrientation().firstAngle);
    }   // close getZAngle method

    /**
     * Method updateValues
     *  -   Prints the values of a number of parameters to the phone
     * @param action    -   Tells the user what method / action that the program is providing data for
     *
     */
    public void updateValues(String action, double initZ, double theta, double currentZ, double zCorrection){
        opMode.telemetry.addData("Current Action = ", action);
        opMode.telemetry.addData("InitZ/targetAngle value  = ", initZ);
        opMode.telemetry.addData("Theta/lastError Value= ", theta);
        opMode.telemetry.addData("CurrentZ/Error Value = ", currentZ);
        opMode.telemetry.addData("zCorrection/derivative Value = ", zCorrection);

        opMode.telemetry.addData("Right Front = ", RF);
        opMode.telemetry.addData("Left Front = ", LF);
        opMode.telemetry.addData("Left Rear = ", LR);
        opMode.telemetry.addData("Right Rear = ", RR);
        opMode.telemetry.update();
    }   // close updateValues method

    /**
     * Method: calcDistance
     * -    Calculates the distance that the robot has traveled based on starting values.
     * Note: These calculations work when driving forward for strafing at 90 degree angles.
     * @param heading   - indicates the direction the robot is angled/heading
     * @param rfStart   - Right Front starting encoder value
     * @param rrStart   - Right Rear starting encoder value
     * @param lfStart   - Left Front starting encoder value
     * @param lrStart   - Left Rear starting encoder value
     */
    public double calcDistance(double heading, double rfStart, double rrStart, double lfStart, double lrStart){

        double strafeFactor = 1;
        double distanceTraveled = 0;
        double rfEncoder = 0;
        double lfEncoder = 0;
        double rrEncoder = 0;
        double lrEncoder = 0;

        rfEncoder = robot.motorRF.getCurrentPosition();
        lfEncoder = robot.motorLF.getCurrentPosition();
        rrEncoder = robot.motorRR.getCurrentPosition();
        lrEncoder = robot.motorLR.getCurrentPosition();

        if ((heading == 90) || (heading == -90)){
            strafeFactor = robot.STRAFE_FACTOR;
        }

        distanceTraveled = ((Math.abs(rfStart - rfEncoder) + Math.abs(lfStart - lfEncoder)
                + Math.abs(rrStart - rrEncoder) + Math.abs(lrStart - lrEncoder)) / 4) / (robot.DRIVE_TICKS_PER_INCH);

        return Math.abs(distanceTraveled * strafeFactor);
    }

    /**
     * Method convertToInches
     *  -   Converts the measured encoder values to inches
     *      Assumes drive motors only
     * @param convValue
     * @return
     */
    private double convertToInches(double convValue){

        return (convValue / robot.USD_COUNTS_PER_INCH);

    }   // end of returnInches method


}   // close the driveMecanum class