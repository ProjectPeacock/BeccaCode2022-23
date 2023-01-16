package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.AutoClass;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto: Blue Terminal", group = "Competition")
public class BlueTerminalAuto extends LinearOpMode {
    public final static HWProfile robot = new HWProfile();
    private LinearOpMode myOpmode=this;

    AutoClass liftControl = new AutoClass(robot,myOpmode);
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose= new Pose2d(-64,-35.25);
        drive.setPoseEstimate(startPose);

        TrajectorySequence align1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-36,-30.5),Math.toRadians(30))

                .build();
        TrajectorySequence score1 = drive.trajectorySequenceBuilder(align1.end())
                .forward(4)
                .waitSeconds(.250)
                .build();
        TrajectorySequence goToCones1=drive.trajectorySequenceBuilder(score1.end())
                .back(12)
                .build();
        TrajectorySequence goToCones2=drive.trajectorySequenceBuilder(goToCones1.end())
                .splineTo(new Vector2d(-16,-36),Math.toRadians(270))
                .splineTo(new Vector2d(-11.5,-59.5),Math.toRadians(270))
                .build();

        waitForStart();

        if(isStopRequested()) return;
        robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
        drive.followTrajectorySequence(align1);
        liftControl.moveLift(2);
        drive.followTrajectorySequence(score1);
        robot.servoGrabber.setPosition(robot.CLAW_OPEN);
        liftControl.moveLift(0);
        drive.followTrajectorySequence(goToCones1);
        drive.followTrajectorySequence(goToCones2);

    }
}
