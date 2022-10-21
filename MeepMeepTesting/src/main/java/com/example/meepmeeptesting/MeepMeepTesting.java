package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(54.669, 54.669, 14.9982, Math.toRadians(321.2592), 11.15)
                .setDimensions(13,13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -63.5, Math.toRadians(90)))
                                .forward(63.5)
                                .waitSeconds(3)
                                .lineToLinearHeading(new Pose2d(36,-12, Math.toRadians(0)))
                                .forward(24)
                                .waitSeconds(1)
                                .back(12)
                                .waitSeconds(1)
                                .forward(12)
                                .waitSeconds(1)
                                .back(60)
                                .waitSeconds(1)
                                .forward(60)
                                .waitSeconds(1)
                                .back(84)
                                .waitSeconds(3)
                                .forward(84)
                                .waitSeconds(1)
                                .back(24)
                                //.waitSeconds(1)
                                //.strafeLeft(24)
                                //.back(12)
                                //.waitSeconds(2)
                                //.forward(12)
                                //.strafeRight(24)
                                //.waitSeconds(1)
                                //.strafeLeft(48)
                                //.back(12)
                                //.waitSeconds(3)
                                //.back(12)
                                //.strafeRight(24)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}