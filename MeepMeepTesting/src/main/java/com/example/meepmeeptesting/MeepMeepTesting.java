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
                .setConstraints(46, 50, Math.toRadians(180), Math.toRadians(222), 13.24)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8.5, -35.4, Math.toRadians(90)))

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(38, -27, Math.toRadians(0.00)), Math.toRadians(90.00))
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(38, -37.00, Math.toRadians(-70.00)))
                        .splineToLinearHeading(new Pose2d(46.5, -27, Math.toRadians(0.00)), Math.toRadians(90.00))
                        .lineToLinearHeading(new Pose2d(46.5, -37.00, Math.toRadians(-70.00)))
                        .splineToLinearHeading(new Pose2d(55, -27, Math.toRadians(0.00)), Math.toRadians(00.00))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(52, -37, Math.toRadians(-90.00)), Math.toRadians(-88.32))
                        .setReversed(false)

                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}