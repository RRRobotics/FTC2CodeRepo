package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.MAX_POSITION;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.*;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Timer;

@Autonomous(name = "RRAutoOdometry")
public class RRAutoOdometry extends LinearOpMode {

    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor arm;
    Servo Extend;
    Servo Pivot;
    CRServo Intake;
    private double speed_factor = 0.4;
    private final int GRAB_POSITION = -219;

    private final int MAX_POSITION = -2092;

    private final int SCORED_POSITION = -1324;

    public static Pose2d endPose;

    enum State {
        IDLE, SCORE_FIRST, SCORING_FIRST, PUSH_OTHERS, SCORING, DRIVE_TO_SCORE_FIRST, DRIVE_TO_SCORE_SECOND, SCORING_SECOND, PARK, RETURN
    }

    State currentState = State.IDLE;



    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        arm = hardwareMap.get(DcMotor.class, "arm");
        Extend = hardwareMap.get(Servo.class, "Extend");
        Pivot = hardwareMap.get(Servo.class, "Pivot");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        telemetry.addData("initialization:", "is a success");
        telemetry.update();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        endPose = new Pose2d();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence scoreFirst = drive.trajectorySequenceBuilder(new Pose2d(8.50, -66.35, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(8.50, -36.00, Math.toRadians(90.00)))
                .build();


        drive.setPoseEstimate(scoreFirst.start());

        TrajectorySequence pushOthers = drive.trajectorySequenceBuilder(new Pose2d(8.50, -36.00, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36, -28.96, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(36, -23, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(49.00, -23), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(49.00, -56), Math.toRadians(-88.30))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(40, -23), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(59, -23), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(59, -55.03), Math.toRadians(270.00))
                .splineToLinearHeading(new Pose2d(49.40, -45.84, Math.toRadians(270)), Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(49.40, -60.00, Math.toRadians(270.00)), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(49.40, -67.00), Math.toRadians(270))
                .build();




        TrajectorySequence driveToScore = drive.trajectorySequenceBuilder(new Pose2d(49.40, -67, Math.toRadians(270.00)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(1.5, -41.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .setReversed(false)
                .build();

        TrajectorySequence driveToScoreSecond = drive.trajectorySequenceBuilder(new Pose2d(49.40, -67, Math.toRadians(270.00)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-1, -41.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .setReversed(false)
                .build();


        TrajectorySequence returnToZone = drive.trajectorySequenceBuilder(new Pose2d(1.5, -41.00, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(49.40, -60, Math.toRadians(270.00)), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(49.40, -65), Math.toRadians(270))
                .setReversed(false)
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-1, -41.00, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(49.40, -65))
                .setVelConstraint(new MecanumVelocityConstraint(1000, 13.24))
                .build();




        ElapsedTime waitTimer = new ElapsedTime();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequenceAsync(scoreFirst);
        currentState = State.SCORE_FIRST;


        setArmPosition(MAX_POSITION);

        
        while(opModeIsActive() && !isStopRequested()) {

            switch(currentState) {
                case SCORE_FIRST:
                    if (!drive.isBusy()) {
                        currentState = State.SCORING_FIRST;
                        setArmPosition(SCORED_POSITION);
                    }
                    break;
                case SCORING_FIRST:
                    if (atArmPosition(SCORED_POSITION)) {
                        currentState = State.PUSH_OTHERS;
                        drive.followTrajectorySequenceAsync(pushOthers);
                        setArmPosition(GRAB_POSITION);
                    }
                    break;
                case PUSH_OTHERS:
                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_SCORE_FIRST;
                        drive.followTrajectorySequenceAsync(driveToScore);
                        setArmPosition(MAX_POSITION);
                    }
                    break;
                case DRIVE_TO_SCORE_FIRST:
                    if (!drive.isBusy()) {
                        currentState = State.SCORING;
                        setArmPosition(SCORED_POSITION);
                    }
                    break;
                case SCORING:
                    if (atArmPosition(SCORED_POSITION)) {
//                        count++;
//                        if (count < 4) {
                            currentState = State.RETURN;
                            drive.followTrajectorySequenceAsync(returnToZone);
                            setArmPosition(GRAB_POSITION);
//                        } else {
//                            currentState = State.IDLE;
//                            setArmPosition(0);
//                        }
                    }
                    break;
                    //for 60 point: return, drive to score 2, scoring
                case RETURN:
                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_SCORE_SECOND;
                        drive.followTrajectorySequenceAsync(driveToScoreSecond);
                        setArmPosition(MAX_POSITION);
                    }
                    break;
                case DRIVE_TO_SCORE_SECOND:
                    if (!drive.isBusy()) {
                        currentState = State.SCORING_SECOND;
                        setArmPosition(GRAB_POSITION);
                    }
                    break;
                case SCORING_SECOND:
                    if (atArmPosition(SCORED_POSITION)) {
                        currentState = State.PARK;
                        drive.followTrajectorySequenceAsync(park);
                        setArmPosition(0);
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                case IDLE:
                    break;
            }

            drive.update();

        }
        endPose = drive.getPoseEstimate();

    }

    public void setArmPosition(int position) {
        setArmPosition(position, 1);
    }
    public void setArmPosition(int position, double power) {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
        arm.setTargetPosition(position);
    }

    public boolean atArmPosition(int target) {
        return Math.abs(target - arm.getCurrentPosition()) < 10;
    }

}
