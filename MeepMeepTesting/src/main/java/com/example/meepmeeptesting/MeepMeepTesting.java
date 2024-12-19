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
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(222), 13.24)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8.50, -66.35, Math.toRadians(90.00)))
                        .lineToSplineHeading(new Pose2d(8.50, -36.00, Math.toRadians(90.00)))

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(38, -28.96, Math.toRadians(90.00)), Math.toRadians(90.00))
                        .splineToLinearHeading(new Pose2d(38, -18, Math.toRadians(90.00)), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(49.00, -18), Math.toRadians(270.00))
                        .splineToConstantHeading(new Vector2d(49.00, -50), Math.toRadians(-88.30))
                        .splineToConstantHeading(new Vector2d(40, -18), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(62, -18), Math.toRadians(270.00))
                        .splineToConstantHeading(new Vector2d(62, -50), Math.toRadians(270.00))
                        .splineToConstantHeading(new Vector2d(52, -18), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(68, -18), Math.toRadians(270.00))
                        .splineToConstantHeading(new Vector2d(64, -63), Math.toRadians(270.00))
                        .setReversed(false)

                        .lineToLinearHeading(new Pose2d(6.5, -39.00, Math.toRadians(90.00)))

                        .lineToConstantHeading(new Vector2d(25, -44.52))
                        .splineToConstantHeading(new Vector2d(40.00, -63), Math.toRadians(270.00))

                        .lineToLinearHeading(new Pose2d(6.5, -39.00, Math.toRadians(90.00)))

                        .lineToConstantHeading(new Vector2d(25, -44.52))
                        .splineToConstantHeading(new Vector2d(40.00, -63), Math.toRadians(270.00))

                        .lineToLinearHeading(new Pose2d(6.5, -39.00, Math.toRadians(90.00)))

                        .lineToConstantHeading(new Vector2d(25, -44.52))
                        .splineToConstantHeading(new Vector2d(40.00, -63), Math.toRadians(270.00))

                        .lineToLinearHeading(new Pose2d(6.5, -39.00, Math.toRadians(90.00)))

                        .lineToConstantHeading(new Vector2d(45.00, -55.00))
                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}