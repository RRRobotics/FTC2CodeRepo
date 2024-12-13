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
    private final int GRAB_POSITION = -135;
    private final int MAX_POSITION = -2092;
    private final int SCORED_POSITION = -1400;


    public static Pose2d endPose;

    enum State {
        IDLE, SCORE_FIRST, SCORING_FIRST, PUSH_OTHERS, SCORING_SECOND, SCORING, DRIVE_TO_SCORE_FIRST, DRIVE_TO_SCORE_SECOND, PARK, RETURN
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
                .lineToLinearHeading(new Pose2d(38.93, -40.00, Math.toRadians(70.00)))
                .turn(-Math.toRadians(140))
                .lineToLinearHeading(new Pose2d(48.59, -40.00, Math.toRadians(70)))
                .turn(-Math.toRadians(140))
                .splineToLinearHeading(new Pose2d(40.00, -67, Math.toRadians(270)), Math.toRadians(270))
                .build();




        TrajectorySequence driveToScore = drive.trajectorySequenceBuilder(new Pose2d(49.40, -67, Math.toRadians(270.00)))
                .lineToLinearHeading(new Pose2d(1.5, -41.00, Math.toRadians(90.00)))
                .build();


        TrajectorySequence returnToZone = drive.trajectorySequenceBuilder(new Pose2d(1.5, -41.00, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(24.33, -44.52, Math.toRadians(-10)))
                .splineToSplineHeading(new Pose2d(40.00, -65, Math.toRadians(270)), Math.toRadians(270.00))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-1, -41.00, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(45.00, -55.00))
                .build();


        ElapsedTime waitTimer = new ElapsedTime();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequenceAsync(scoreFirst);
        currentState = State.SCORE_FIRST;

        setArmPosition(MAX_POSITION);

        int samplesScored = 0;
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
                        drive.followTrajectorySequenceAsync(driveToScore);
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
