package org.firstinspires.ftc.teamcode.Libs;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

public class AutoThreadClass implements Runnable{

    //Thread run condition;
    private boolean isRunning = true;
    public double currentLiftPosition=0;

    private HWProfile robot;
    public double RF, LF, LR, RR;
    public OpMode opMode;
    ElapsedTime runTime = new ElapsedTime();
    public int cyclesRun=0;
    public int targetPos=0;
    public double servoPos=0.3;
    AutoParams params = new AutoParams();


    /*
     * Constructor method
     */
    public AutoThreadClass(HWProfile myRobot, OpMode myOpMode){
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
        if(pos==0){
            this.targetPos= robot.LIFT_BOTTOM;
        }else if(pos==1){
            this.targetPos= robot.LIFT_LOW;
        }else if(pos==2){
            this.targetPos= robot.LIFT_MID;
        }else if(pos==3){
            this.targetPos= robot.LIFT_HIGH;
        }
    }

    public void controlLift(){
        this.currentLiftPosition=robot.winch.getPositions().get(0);
    }   //  end of method controlLift

    /*
     * Method: stopLiftThread
     *  -   Turns off the shooter control process
     */
    public void stopLiftThread(){

        robot.winch.stopMotor();
        controlLift();
        this.isRunning = false;
    }   //  end of stopShooterThread method
    //method for moving lift to retrieve cones
    //cyclesRun corresponds to how many cycles have been completed. This class keeps track of how many cycles have been completed internally
    public void moveLiftGrab(){
        if(cyclesRun==0){
            this.targetPos= params.cycle1;
        }else if(cyclesRun==1){
            this.targetPos= params.cycle2;
        }else if(cyclesRun==2){
            this.targetPos= params.cycle3;
        }else if(cyclesRun==3){
            this.targetPos= params.cycle4;
        }else if(cyclesRun==4){
            this.targetPos= params.cycle5;
        }
        cyclesRun++;
    }

    public void update(){
        robot.winch.setTargetPosition(this.targetPos);
        robot.servoGrabber.setPosition(this.servoPos);
        if(!robot.winch.atTargetPosition()){
            robot.winch.set(1);
        }else{
            robot.winch.stopMotor();
        }

    }

    //claw control methods
    public void openClaw(){
        this.servoPos=robot.CLAW_OPEN;
    }
    public void closeClaw(){
        this.servoPos=robot.CLAW_CLOSE;
    }

    public void run(){
        while(isRunning){
            update();
            try{
                Thread.sleep(100);
            } catch (InterruptedException e){
                e.printStackTrace();
            }   //end of try

        }       // end of while(isRunning)
    }   // end of method run()

}   // close the AutoClass class