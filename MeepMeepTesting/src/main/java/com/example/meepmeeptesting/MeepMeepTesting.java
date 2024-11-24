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

                        .lineToSplineHeading(new Pose2d(8.50, -33.00, Math.toRadians(90.00)))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(33, -28.96), Math.toRadians(90.00))
                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(34, -15.57), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(47.00, -9.23), Math.toRadians(270.00))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(47.00, -54.98), Math.toRadians(-88.30))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(49.40, -41.02, Math.toRadians(90.00)), Math.toRadians(90.00))
                        .turn(Math.toRadians(180))
                        .lineToConstantHeading(new Vector2d(49.40, -56))
                        .lineToSplineHeading(new Pose2d(4.18, -33.00, Math.toRadians(90.00 + 1e-6)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(49.40, -56, Math.toRadians(270.00)), Math.toRadians(270.00))
                        .setReversed(false)

                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}