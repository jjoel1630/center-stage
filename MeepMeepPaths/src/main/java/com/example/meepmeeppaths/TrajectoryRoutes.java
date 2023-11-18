package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TrajectoryRoutes {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-36.67, -63.54, Math.toRadians(90.00)))
//                                .splineToConstantHeading(new Vector2d(-47.36, -38.30), Math.toRadians(90.00))
//                                .lineToLinearHeading(new Pose2d(-47.36, -49.88, Math.toRadians(90.00)))
//                                .build()
                                drive.trajectorySequenceBuilder(new Pose2d(-47.36, -49.88, Math.toRadians(90.00)))
                                        .splineTo(new Vector2d(-15.29, -36.37), Math.toRadians(0.00))
                                        .splineTo(new Vector2d(50.47, -38.00), Math.toRadians(0.00))
                                        .build()
                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}