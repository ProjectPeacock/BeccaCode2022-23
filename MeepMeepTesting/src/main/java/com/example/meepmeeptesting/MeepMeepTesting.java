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
        MeepMeep meepMeep = new MeepMeep(600).setBackground(ImageIO.read(new URL("E:/FTC/2022-2023/BeccaCode2022-23/PowerPlayField.png")));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(89.236, 90, 17.927212540948645, Math.toRadians(360), 11.42)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-64,-35.25,Math.toRadians(0)))
                                //initial alignment with mid pole
                                .splineTo(new Vector2d(-36,-30.5),Math.toRadians(30))

                                //final alignment with mid pole
                                .forward(4)

                                //wait for claw to open
                                .waitSeconds(0.25)

                                //leave main pole
                                .back(8)

                                //orient to avoid poles
                                .turn(Math.toRadians(-60))

                                //drive to cone stack
                                .splineTo(new Vector2d(-12,-45),Math.toRadians(270))
                                .forward(14.5)

                                //retrieve cone
                                .waitSeconds(0.5)

                                // cycle 1
                                .back(12)
                                .splineToSplineHeading(new Pose2d(-16,-32.5,Math.toRadians(130)),Math.toRadians(270))
                                .waitSeconds(0.5)
                                .back(6)
                                .splineToSplineHeading(new Pose2d(-12,-45,Math.toRadians(270)),Math.toRadians(270))
                                .forward(14.5)
                                .waitSeconds(0.5)

                                // cycle 2
                                .back(12)
                                .splineToSplineHeading(new Pose2d(-16,-32.5,Math.toRadians(130)),Math.toRadians(270))
                                .waitSeconds(0.5)
                                .back(6)
                                .splineToSplineHeading(new Pose2d(-12,-45,Math.toRadians(270)),Math.toRadians(270))
                                .forward(14.5)
                                .waitSeconds(0.5)

                                // cycle 3
                                .back(12)
                                .splineToSplineHeading(new Pose2d(-16,-32.5,Math.toRadians(130)),Math.toRadians(270))
                                .waitSeconds(0.5)
                                .back(6)
                                .splineToSplineHeading(new Pose2d(-12,-45,Math.toRadians(270)),Math.toRadians(270))
                                .forward(14.5)
                                .waitSeconds(0.5)

                                // cycle 4
                                .back(12)
                                .splineToSplineHeading(new Pose2d(-16,-32.5,Math.toRadians(130)),Math.toRadians(270))
                                .waitSeconds(0.5)
                                .back(6)
                                .splineToSplineHeading(new Pose2d(-12,-45,Math.toRadians(270)),Math.toRadians(270))
                                .forward(14.5)
                                .waitSeconds(0.5)

                                // cycle 5
                                .back(12)
                                .splineToSplineHeading(new Pose2d(-8,-30.5,Math.toRadians(40)),Math.toRadians(270))
                                .waitSeconds(0.5)

                                //park 1
                                /*
                                .back(3)
                                .splineToSplineHeading(new Pose2d(-12,-12,Math.toRadians(0)),Math.toRadians(90))
                                 */

                                //park 2
                                .back(8)
                                .turn(Math.toRadians(-40))

                                //park 3
                                /*
                                .back(6)
                                .splineToSplineHeading(new Pose2d(-12,-40,Math.toRadians(270)),Math.toRadians(270))
                                .splineToSplineHeading(new Pose2d(-12,-59.5,Math.toRadians(270)),Math.toRadians(270))
                                 */

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}