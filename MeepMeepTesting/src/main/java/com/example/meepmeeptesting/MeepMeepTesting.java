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
                .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(222), 13.24)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8.50, -66.35, Math.toRadians(90.00)))
                        .lineToSplineHeading(new Pose2d(8.50, -35.50, Math.toRadians(90.00)))

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(38, -28.96, Math.toRadians(90.00)), Math.toRadians(90.00))
                        .splineToLinearHeading(new Pose2d(38, -18, Math.toRadians(90.00)), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(46, -18), Math.toRadians(270.00))
                        .splineToLinearHeading(new Pose2d(42, -45, Math.toRadians(90)), Math.toRadians(-88.30))
                        .splineToConstantHeading(new Vector2d(38, -18), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(52, -18), Math.toRadians(270.00))
                        .splineToConstantHeading(new Vector2d(50, -45), Math.toRadians(270.00))
                        .splineToConstantHeading(new Vector2d(47, -16), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(63, -16), Math.toRadians(270.00))
                        .splineToConstantHeading(new Vector2d(60, -45), Math.toRadians(270.00))
                        .splineToConstantHeading(new Vector2d(60, -30), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(60, -67), Math.toRadians(270))
                        .setReversed(false)

                        .lineToLinearHeading(new Pose2d(-4, -36, Math.toRadians(90.00)))

                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(40.00, -68), Math.toRadians(270.00))
                        .setReversed(false)

                        .lineToLinearHeading(new Pose2d(-2, -35.50, Math.toRadians(90.00)))

                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(40.00, -68), Math.toRadians(270.00))
                        .setReversed(false)

                        .lineToLinearHeading(new Pose2d(-4, -36.00, Math.toRadians(90.00)))

                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(40.00, -68), Math.toRadians(270.00))
                        .setReversed(false)

                        .lineToLinearHeading(new Pose2d(-6, -36.00, Math.toRadians(90.00)))

                        .lineToConstantHeading(new Vector2d(45.00, -60.00))
                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}