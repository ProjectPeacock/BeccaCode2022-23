package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.AutoClass;
import org.firstinspires.ftc.teamcode.Libs.AutoParams;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.List;

@Autonomous(name = "Test Auto RR", group = "Competition")
public class TestAutoRR extends OpMode {
    /*

    OPMODE MAP - PLEASE READ BEFORE EDITING

    This opMode uses TrajectorySequences from RoadRunner. They are made to be run back to back.
    The order of operations is: untilCycle -> firstCycle -> cycles2to4 (repeated 3 times) -> highCycle -> park

    Each parking position is its own TrajectorySequence. They are all made to be run following highCycle.

    All values for target position and heading come from AutoParams.java.

     */

    //lift control init
    public final static HWProfile robot = new HWProfile();
    private OpMode myOpmode=this;
    AutoClass liftControl = new AutoClass(robot,myOpmode);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    private int parkPos = 1;

    //init params
    AutoParams params = new AutoParams();

    public void init() {
        robot.init(hardwareMap);

        Pose2d startPose= new Pose2d(params.startPoseX,params.startPoseY,Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence untilCycle = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(28.5,-31),Math.toRadians(120))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    liftControl.moveLiftScore(2);
                })
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    liftControl.openClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    liftControl.moveLiftGrab();
                })
                .turn(Math.toRadians(-60))
                .splineToLinearHeading(new Pose2d(60,-12,Math.toRadians(0)),Math.toRadians(0))
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    liftControl.closeClaw();
                })
                .build();

        TrajectorySequence cycleMid = drive.trajectorySequenceBuilder(untilCycle.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    liftControl.moveLiftScore(2);
                })
                .back(6)
                .splineToSplineHeading(new Pose2d(30,-17.5,Math.toRadians(220)),Math.toRadians(190))
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    liftControl.openClaw();
                })
                .back(1)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{
                    liftControl.moveLiftGrab();
                })
                .splineToSplineHeading(new Pose2d(60,-12,Math.toRadians(0)),Math.toRadians(0))
                .build();

        TrajectorySequence cycleHigh = drive.trajectorySequenceBuilder(cycleMid.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    liftControl.moveLiftScore(3);
                })
                .back(6)
                .splineToSplineHeading(new Pose2d(30,-6.5,Math.toRadians(130)),Math.toRadians(17220))
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    liftControl.openClaw();
                })
                .back(1)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{
                    liftControl.moveLiftGrab();
                })
                .splineToSplineHeading(new Pose2d(60,-12,Math.toRadians(0)),Math.toRadians(0))
                .build();

        TrajectorySequence finalHigh = drive.trajectorySequenceBuilder(cycleHigh.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    liftControl.moveLiftScore(3);
                })
                .back(6)
                .splineToSplineHeading(new Pose2d(30,-6.5,Math.toRadians(130)),Math.toRadians(17220))
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    liftControl.openClaw();
                })
                .back(1)
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(finalHigh.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    liftControl.moveLiftScore(0);
                })
                .splineToSplineHeading(new Pose2d(12,-12,Math.toRadians(90)),Math.toRadians(180))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(finalHigh.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    liftControl.moveLiftScore(0);
                })
                .splineToSplineHeading(new Pose2d(36,-12,Math.toRadians(90)),Math.toRadians(130))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(finalHigh.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    liftControl.moveLiftScore(0);
                })
                .splineToSplineHeading(new Pose2d(60,-12,Math.toRadians(90)),Math.toRadians(0))
                .build();



        //score preload
        drive.followTrajectorySequenceAsync(untilCycle);
        for(int i=0;i<params.numMidCycles;i++){
            drive.followTrajectorySequenceAsync(cycleMid);
        }
        for(int i=0;i<params.numHighCycles;i++){
            drive.followTrajectorySequenceAsync(cycleHigh);
        }

        drive.followTrajectorySequenceAsync(finalHigh);

        if(parkPos==1){
            drive.followTrajectorySequenceAsync(park1);
        }else if(parkPos==2){
            drive.followTrajectorySequenceAsync(park2);
        }else{
            drive.followTrajectorySequenceAsync(park3);
        }

    }
    public void loop(){
        drive.update();
        liftControl.update();
    }

}
