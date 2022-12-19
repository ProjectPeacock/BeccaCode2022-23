package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "RR Auto: Red Corner", group = "Concept")
@Disabled

public class RR_RedCornerAuto extends LinearOpMode {
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-29,-66,Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory scoreCone1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-27,-55),Math.toRadians(65))
                .UNSTABLE_addDisplacementMarkerOffset(-0.5,()->{})
                .waitSeconds(0.5)
                .setTangent(90)
                .setReversed(true)
                .splineTo(new Vector2d(-29,-66),Math.toRadians(90))

                .build();



        waitForStart();
        while(opModeIsActive()){

        }
    }
}
