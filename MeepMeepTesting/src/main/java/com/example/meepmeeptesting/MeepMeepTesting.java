package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.io.IOException;
import java.net.URL;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) throws IOException {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(89.236, 90, 17.927212540948645, Math.toRadians(360), 11.42)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35.25,-64, Math.toRadians(90)))
                                //drive to mid pole
                                .splineTo(new Vector2d(30.5,-36),Math.toRadians(120))

                                //align to pole
                                .forward(4)

                                //wait for claw to open
                                .waitSeconds(0.25)

                                //leave main pole
                                .back(8)

                                //orient to avoid poles
                                .turn(Math.toRadians(-40))

                                //drive to cone stack
                                .splineTo(new Vector2d(45,-12),Math.toRadians(0))
                                .forward(14.5)

                                //retrieve cone
                                .waitSeconds(0.35)

                                //mid cycle
                                .back(12)
                                .lineToSplineHeading(new Pose2d(24,-10,Math.toRadians(270)))
                                .forward(4)
                                .back(4)
                                .lineToSplineHeading(new Pose2d(56,-12,Math.toRadians(0)))
                                .forward(3.5)
                                .waitSeconds(0.35)

                                .back(12)
                                .lineToSplineHeading(new Pose2d(24,-10,Math.toRadians(270)))
                                .forward(4)
                                .back(4)
                                .lineToSplineHeading(new Pose2d(56,-12,Math.toRadians(0)))
                                .forward(3.5)
                                .waitSeconds(0.35)

                                .back(12)
                                .lineToSplineHeading(new Pose2d(24,-10,Math.toRadians(270)))
                                .forward(4)
                                .back(4)
                                .lineToSplineHeading(new Pose2d(56,-12,Math.toRadians(0)))
                                .forward(3.5)
                                .waitSeconds(0.35)

                                .back(12)
                                .lineToSplineHeading(new Pose2d(24,-10,Math.toRadians(270)))
                                .forward(4)
                                .back(4)
                                .lineToSplineHeading(new Pose2d(56,-12,Math.toRadians(0)))
                                .forward(3.5)
                                .waitSeconds(0.35)


                                //high cycle
                                .back(12)
                                .lineToSplineHeading(new Pose2d(24,-14,Math.toRadians(90)))
                                .forward(4)
                                .back(4)
                                .waitSeconds(0.35)

                                //park 1
                                .strafeLeft(12)

                                //park 3
                                //.strafeRight(12)



                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}