package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
public class TestAutoRR extends LinearOpMode {
    /*

    OPMODE MAP - PLEASE READ BEFORE EDITING

    This opMode uses TrajectorySequences from RoadRunner. They are made to be run back to back.
    The order of operations is: untilCycle -> firstCycle -> cycles2to4 (repeated 3 times) -> highCycle -> park

    Each parking position is its own TrajectorySequence. They are all made to be run following highCycle.

    All values for target position and heading come from AutoParams.java.

     */

    //lift control init
    public final static HWProfile robot = new HWProfile();
    private LinearOpMode myOpmode=this;
    AutoClass liftControl = new AutoClass(robot,myOpmode);

    //init params
    AutoParams params = new AutoParams();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose= new Pose2d(params.startPoseX,params.startPoseY,Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        TrajectorySequence untilCycle = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(12,12,Math.toRadians(90)))
                .build();



        waitForStart();

        if(isStopRequested()) return;

        //score preload
        drive.followTrajectorySequence(untilCycle);

    }

}
