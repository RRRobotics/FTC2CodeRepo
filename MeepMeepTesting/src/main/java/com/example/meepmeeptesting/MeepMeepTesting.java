
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

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 50, Math.toRadians(300), Math.toRadians(250), 13.24)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8.50, -66.35, Math.toRadians(90.00)))
                        .lineToSplineHeading(new Pose2d(8.50, -36.00, Math.toRadians(90.00)))

                        .lineToLinearHeading(new Pose2d(33.00, -50.00, Math.toRadians(70.00)))
                        .turn(-Math.toRadians(140))
                        .lineToLinearHeading(new Pose2d(43.00, -50.00, Math.toRadians(70)))
                        .turn(-Math.toRadians(140))
                        .lineToLinearHeading(new Pose2d(53.00, -50.00, Math.toRadians(70)))
                        .turn(-Math.toRadians(140))
                        .splineToLinearHeading(new Pose2d(40.00, -65, Math.toRadians(270)), Math.toRadians(270))

                        .lineToLinearHeading(new Pose2d(6.5, -39.00, Math.toRadians(270.00)))

                        .lineToConstantHeading(new Vector2d(25, -44.52))
                        .splineToConstantHeading(new Vector2d(40.00, -63), Math.toRadians(270.00))

                        .lineToLinearHeading(new Pose2d(4.5, -39.00, Math.toRadians(270.00)))

                        .lineToConstantHeading(new Vector2d(25, -44.52))
                        .splineToConstantHeading(new Vector2d(40.00, -63), Math.toRadians(270.00))

                        .lineToLinearHeading(new Pose2d(2.5, -39.00, Math.toRadians(270.00)))

                        .lineToConstantHeading(new Vector2d(25, -44.52))
                        .splineToConstantHeading(new Vector2d(40.00, -63), Math.toRadians(270.00))

                        .lineToLinearHeading(new Pose2d(0.5, -39.00, Math.toRadians(270.00)))


                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}