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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8.50, -66.35, Math.toRadians(90.00)))
                        .lineToSplineHeading(new Pose2d(8.52, -34.00, Math.toRadians(90.00)))
                        .setReversed(true)
                        .splineTo(new Vector2d(34.70, -28.96), Math.toRadians(55.67))
                        .splineTo(new Vector2d(36.26, -7.57), Math.toRadians(62.72))
                        .splineTo(new Vector2d(45.99, -9.39), Math.toRadians(270.00))
                        .setReversed(false)
                        .lineTo(new Vector2d(46.23, -56.59))
                        .splineToConstantHeading(new Vector2d(51.29, -11.56), Math.toRadians(59.89))
                        .splineToConstantHeading(new Vector2d(56.35, -19.75), Math.toRadians(-68.89))
                        .splineToConstantHeading(new Vector2d(56.11, -56.59), Math.toRadians(270.00))
                        .splineToConstantHeading(new Vector2d(57.55, -14.93), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(62.38, -13.70), Math.toRadians(270.00))
                        .splineToConstantHeading(new Vector2d(61.88, -56.23), Math.toRadians(90.00))
                        .splineTo(new Vector2d(51.77, -55.34), Math.toRadians(205.95))
                        .splineTo(new Vector2d(49.68, -62.70), Math.toRadians(270.00))
                        .splineTo(new Vector2d(50.09, -28.70), Math.toRadians(89.31))
                        .lineToSplineHeading(new Pose2d(3.18, -32.99, Math.toRadians(90.00)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(49.68, -62.70, Math.toRadians(270.00)), Math.toRadians(270.00))
                        .setReversed(false)
                        .lineToSplineHeading(new Pose2d(3.18, -32.99, Math.toRadians(90.00)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(49.68, -62.70, Math.toRadians(270.00)), Math.toRadians(270.00))
                        .setReversed(false)
                        .lineToSplineHeading(new Pose2d(3.18, -32.99, Math.toRadians(90.00)))

                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}