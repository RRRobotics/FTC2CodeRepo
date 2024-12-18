
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        int y = -40;
        double turn = Math.toRadians(70);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 50, Math.toRadians(50), Math.toRadians(250), 13.24)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8.50, -66.35, Math.toRadians(90.00)))
                        .lineToSplineHeading(new Pose2d(8.50, -36.00, Math.toRadians(90.00)))

                        .lineToLinearHeading(new Pose2d(38.00, y, turn / 2.0))
                        .lineToLinearHeading(new Pose2d(38.00, y - 10, -turn / 2.0))
                        .lineToLinearHeading(new Pose2d(48.00, y, turn / 2.0))
                        .lineToLinearHeading(new Pose2d(48.00, y - 10, -turn / 2.0))
                        .lineToLinearHeading(new Pose2d(58.00, y, turn / 2.0))
                        .lineToSplineHeading(new Pose2d(58.00, -65, Math.toRadians(270)))

                        .lineToLinearHeading(new Pose2d(6.5, -39.00, Math.toRadians(270.00)))

                        .lineToConstantHeading(new Vector2d(25, -44.52))
                        .splineToConstantHeading(new Vector2d(40.00, -63), Math.toRadians(270.00))

                        .lineToLinearHeading(new Pose2d(6.5, -39.00, Math.toRadians(270.00)))

                        .lineToConstantHeading(new Vector2d(25, -44.52))
                        .splineToConstantHeading(new Vector2d(40.00, -63), Math.toRadians(270.00))

                        .lineToLinearHeading(new Pose2d(6.5, -39.00, Math.toRadians(270.00)))

                        .lineToConstantHeading(new Vector2d(25, -44.52))
                        .splineToConstantHeading(new Vector2d(40.00, -63), Math.toRadians(270.00))

                        .lineToLinearHeading(new Pose2d(6.5, -39.00, Math.toRadians(270.00)))


                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}