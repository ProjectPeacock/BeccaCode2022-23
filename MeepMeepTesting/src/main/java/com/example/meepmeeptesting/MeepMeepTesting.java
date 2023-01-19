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
                .setConstraints(45, 45, 17.927212540948645, Math.toRadians(360), 11.42)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35.25,-64, Math.toRadians(90)))
                                //drive to mid pole
                                .splineTo(new Vector2d(28.5,-31),Math.toRadians(120))
                                .waitSeconds(0.25)
                                .turn(Math.toRadians(-60))
                                .splineToLinearHeading(new Pose2d(60,-12,Math.toRadians(0)),Math.toRadians(0))
                                .waitSeconds(0.25)
                                .back(6)
                                .splineToSplineHeading(new Pose2d(30,-17.5,Math.toRadians(220)),Math.toRadians(190))

                                .waitSeconds(0.25)
                                .back(1)
                                .splineToSplineHeading(new Pose2d(60,-12,Math.toRadians(0)),Math.toRadians(0))

                                .waitSeconds(0.25)
                                .back(6)
                                .splineToSplineHeading(new Pose2d(30,-6.5,Math.toRadians(130)),Math.toRadians(170))

                                .waitSeconds(0.25)
                                .back(1)
                                .splineToSplineHeading(new Pose2d(60,-12,Math.toRadians(0)),Math.toRadians(0))

                                .waitSeconds(0.25)
                                .back(6)
                                .splineToSplineHeading(new Pose2d(30,-6.5,Math.toRadians(130)),Math.toRadians(170))

                                .waitSeconds(0.25)
                                .back(1)
                                .splineToSplineHeading(new Pose2d(60,-12,Math.toRadians(0)),Math.toRadians(0))

                                .waitSeconds(0.25)
                                .back(6)
                                .splineToSplineHeading(new Pose2d(30,-6.5,Math.toRadians(130)),Math.toRadians(170))

                                .waitSeconds(0.25)
                                .back(1)
                                .splineToSplineHeading(new Pose2d(60,-12,Math.toRadians(0)),Math.toRadians(0))

                                .waitSeconds(0.25)
                                .back(6)
                                .splineToSplineHeading(new Pose2d(30,-6.5,Math.toRadians(130)),Math.toRadians(170))

                                .waitSeconds(0.25)
                                .back(1)

                                .splineToSplineHeading(new Pose2d(60,-12,Math.toRadians(90)),Math.toRadians(0))




                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}