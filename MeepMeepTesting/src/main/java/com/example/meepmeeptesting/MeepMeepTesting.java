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
                .setConstraints(50, 45, Math.toRadians(180), Math.toRadians(222), 13.24)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8.50, -66.35, Math.toRadians(90.00)))
                        .lineToSplineHeading(new Pose2d(8.50, -34.00, Math.toRadians(90.00)))

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(36, -28.96, Math.toRadians(90.00)), Math.toRadians(90.00))
                        .splineToLinearHeading(new Pose2d(36, -15, Math.toRadians(90.00)), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(51.00, -19), Math.toRadians(270.00))
                        .splineToConstantHeading(new Vector2d(47.00, -56), Math.toRadians(-88.30))
                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(44.12, -15), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(55, -15), Math.toRadians(270.00))
                        .splineToConstantHeading(new Vector2d(55, -55.03), Math.toRadians(270.00))
                        .splineToLinearHeading(new Pose2d(49.40, -45.84, Math.toRadians(270)), Math.toRadians(225))
                        .splineToLinearHeading(new Pose2d(49.40, -63.00, Math.toRadians(270.00)), Math.toRadians(270))

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(1.5, -40.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                        .setReversed(false)

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(49.40, -60, Math.toRadians(270.00)),Math.toRadians(270.00))
                        .splineToConstantHeading(new Vector2d(49.4, -68), Math.toRadians(270))
                        .setReversed(false)

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(0, -40.00, Math.toRadians(90.00)), Math.toRadians(90.00))

                        .setReversed(false)

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(49.40, -68, Math.toRadians(270.00)), Math.toRadians(270.00))
                        .setReversed(false)

                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}