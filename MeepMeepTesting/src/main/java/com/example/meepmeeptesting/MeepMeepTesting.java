package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.2, 50, Math.toRadians(180), Math.toRadians(222), 13.24)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8.50, -34.00, Math.toRadians(90.00)))

                        .splineToConstantHeading(new Vector2d(34.70, -28.96), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(36.26, -7.57), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(49.00, -9.23), Math.toRadians(-80.52))
                        .splineToConstantHeading(new Vector2d(49.00, -54.98), Math.toRadians(-88.30))
                        .splineToLinearHeading(new Pose2d(49.14, -41.02, Math.toRadians(90.00)), Math.toRadians(90.00))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(49.14, -60.83, Math.toRadians(270.00)), Math.toRadians(270.00))
                        .setReversed(false)

                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}