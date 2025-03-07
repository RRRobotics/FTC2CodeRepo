package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(53, 50, Math.toRadians(180), Math.toRadians(222), 13.24)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8.5, -40, Math.toRadians(90)))

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(36.5, -28, Math.toRadians(0.00)), Math.toRadians(90))
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(36.5, -37.00, Math.toRadians(-70.00)))
                        .splineToLinearHeading(new Pose2d(46.25, -27, Math.toRadians(0.00)), Math.toRadians(50.00))
                        .turn(0.01)
                        .lineToLinearHeading(new Pose2d(46.25, -37.00, Math.toRadians(-70.00)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(63, -15.19, Math.toRadians(90.00)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(63, -45), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(50, -65.5), Math.toRadians(270))

                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}