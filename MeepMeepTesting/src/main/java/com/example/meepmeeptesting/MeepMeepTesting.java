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

                        .splineToLinearHeading(new Pose2d(38, -32, Math.toRadians(90.00)), Math.toRadians(90.00))
                        .splineToLinearHeading(new Pose2d(37, -18, Math.toRadians(90.00)), Math.toRadians(90.00))
                        .splineToLinearHeading(new Pose2d(46, -18, Math.toRadians(90)), Math.toRadians(270.00))
                        .splineToLinearHeading(new Pose2d(42, -47, Math.toRadians(90)), Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(37, -18, Math.toRadians(90)), Math.toRadians(90.00))
                        .splineToLinearHeading(new Pose2d(53, -18, Math.toRadians(90)), Math.toRadians(270.00))
                        .splineToLinearHeading(new Pose2d(50, -48, Math.toRadians(90)), Math.toRadians(270.00))
                        .splineToLinearHeading(new Pose2d(44, -15, Math.toRadians(90)), Math.toRadians(90.00))
                        .splineToLinearHeading(new Pose2d(60, -15, Math.toRadians(90)), Math.toRadians(270.00))
                        .splineToLinearHeading(new Pose2d(62, -20, Math.toRadians(90)), Math.toRadians(270.00))
                        .splineToLinearHeading(new Pose2d(62, -45, Math.toRadians(90)), Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(62, -50, Math.toRadians(90)), Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(40, -68, Math.toRadians(90)), Math.toRadians(270))
                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}