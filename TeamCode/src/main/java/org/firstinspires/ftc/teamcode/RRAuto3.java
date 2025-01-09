package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.MAX_POSITION;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.*;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import net.jafama.FastMath;

import java.text.DecimalFormat;


@Autonomous(name = "RRAuto3")
public class RRAuto3 extends LinearOpMode {

    DcMotor FR, FL, BR, BL;
    DcMotor arm, extend, flip;
    Servo pitch, heading, push, bumper;
    private double speed_factor = 0.4;
    private final int GRAB_POSITION = -125;

    private final int MAX_POSITION = -2010;

    private final int SCORED_POSITION = -1350;

    //opposite of teleop
    private final int FLIP_SCORE = 0;
    private final int FLIP_INTAKE = -1400;

    public static Pose2d endPose;

    enum State {
        IDLE, SCORE_FIRST, SCORING_FIRST, PUSH_OTHERS, SCORING, DRIVE_TO_SCORE, PARK, RETURN
    }

    State currentState = State.IDLE;



    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        arm = hardwareMap.get(DcMotor.class, "arm");
        extend = hardwareMap.get(DcMotor.class, "extend");
        flip = hardwareMap.get(DcMotor.class, "flip");
        heading = hardwareMap.get(Servo.class, "heading");
        pitch = hardwareMap.get(Servo.class, "pitch");
        push = hardwareMap.get(Servo.class, "push");
        bumper = hardwareMap.get(Servo.class, "bumper");
        telemetry.addData("initialization:", "is a success");
        telemetry.update();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        endPose = new Pose2d();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence scoreFirst = drive.trajectorySequenceBuilder(new Pose2d(8.50, -66.35, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(8.50, -35.50, Math.toRadians(90.00)))
                .addDisplacementMarker(29.7, () -> {setArmPosition(SCORED_POSITION);})
                .build();

        drive.setPoseEstimate(scoreFirst.start());


        TrajectorySequence pushOthers = drive.trajectorySequenceBuilder(new Pose2d(8.5, -35.5, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(37, -28.96, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(37, -18, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(46, -18), Math.toRadians(270.00))
                .splineToLinearHeading(new Pose2d(42, -47, Math.toRadians(90)), Math.toRadians(-88.30))
                .splineToConstantHeading(new Vector2d(37, -18), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(53, -18), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(50, -47), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(47, -16), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(63, -16), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(63, -50), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(60, -30), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(60, -68), Math.toRadians(270))
                .setReversed(false)
                .addTemporalMarker(9, () -> {setArmPosition(MAX_POSITION);})
                .build();

        TrajectorySequence driveToScoreFar = drive.trajectorySequenceBuilder(new Pose2d(60.00, -66.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-4, -36.5, Math.toRadians(90.00)))
                .addDisplacementMarker(70, () -> {setArmPosition(SCORED_POSITION);})
                .addDisplacementMarker(5, () -> {flip.setTargetPosition(FLIP_SCORE);})
                .addDisplacementMarker(0.5, () -> {setArmPosition(MAX_POSITION);})
                .build();

        TrajectorySequence driveToScoreThird = drive.trajectorySequenceBuilder(new Pose2d(40.00, -68.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-2, -36, Math.toRadians(90.00)))
                .addDisplacementMarker(52, () -> {setArmPosition(SCORED_POSITION);})
                .addDisplacementMarker(5, () -> {flip.setTargetPosition(FLIP_SCORE);})
                .addDisplacementMarker(0.5, () -> {setArmPosition(MAX_POSITION);})
                .build();

        TrajectorySequence driveToScoreFourth = drive.trajectorySequenceBuilder(new Pose2d(40.00, -68.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-4, -36, Math.toRadians(90.00)))
                .addDisplacementMarker(53.5, () -> {setArmPosition(SCORED_POSITION);})
                .addDisplacementMarker(5, () -> {flip.setTargetPosition(FLIP_SCORE);})
                .addDisplacementMarker(0.5, () -> {setArmPosition(MAX_POSITION);})
                .build();

        TrajectorySequence driveToScoreFifth = drive.trajectorySequenceBuilder(new Pose2d(40.00, -68.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-3.5, -35.5, Math.toRadians(90.00)))
                .addDisplacementMarker(52, () -> {setArmPosition(SCORED_POSITION);})
                .addDisplacementMarker(5, () -> {flip.setTargetPosition(FLIP_SCORE);})
                .addDisplacementMarker(0.5, () -> {setArmPosition(MAX_POSITION);})
                .build();

        TrajectorySequence returnToZone = drive.trajectorySequenceBuilder(new Pose2d(-4, -37, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(40.00, -70.5), Math.toRadians(270.00))
                .setReversed(false)
                .addDisplacementMarker(5, () -> {flip.setTargetPosition(FLIP_INTAKE);})
                .addTemporalMarker(-1.5, () -> {setArmPosition(SCORED_POSITION);})
                .build();

        TrajectorySequence returnToZoneLater = drive.trajectorySequenceBuilder(new Pose2d(-4, -37, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(40.00, -71), Math.toRadians(270.00))
                .setReversed(false)
                .addDisplacementMarker(5, () -> {flip.setTargetPosition(FLIP_INTAKE);})
                .addTemporalMarker(-3, () -> {setArmPosition(SCORED_POSITION);})
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(22.75, -55.5, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(40.00, -68), Math.toRadians(270.00))
                .setReversed(false)
                .build();


        waitForStart();

        heading.setPosition(0.52);
        extend.setTargetPosition(0);
        flip.setTargetPosition(0);
        flip.setPower(1);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setTargetPosition(0);
        push.setPosition(0);
        bumper.setPosition(1);
        pitch.setPosition(0);

        if(isStopRequested()) return;

        drive.followTrajectorySequenceAsync(scoreFirst);
        currentState = State.SCORE_FIRST;

        setArmPosition(MAX_POSITION);

        ElapsedTime timer = new ElapsedTime();

        int samplesScored = 0;
        double robotHeading, servoHeading;
        boolean intaking;


        while(opModeIsActive() && !isStopRequested()) {
            switch(currentState) {
                case SCORE_FIRST:
                    if (!drive.isBusy()) {
                        currentState = State.SCORING_FIRST;
                        setArmPosition(SCORED_POSITION);
                        timer.reset();
                    }
                    break;
                case SCORING_FIRST:
                    if (timer.milliseconds() > 100) {
                        currentState = State.PUSH_OTHERS;
                        drive.followTrajectorySequenceAsync(pushOthers);
                        setArmPosition(-600);
                        flip.setTargetPosition(FLIP_INTAKE);
                        samplesScored++;
                        bumper.setPosition(0);
                    }
                    break;
                case PUSH_OTHERS:
                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_SCORE;
                        drive.followTrajectorySequenceAsync(driveToScoreFar);
                        bumper.setPosition(1);
                    } else if (atFlipPosition(FLIP_INTAKE)) {
                        setArmPosition(GRAB_POSITION);
                    }
                    break;
                case DRIVE_TO_SCORE:
                    if (!drive.isBusy()) {
                        currentState = State.SCORING;
                        setArmPosition(SCORED_POSITION);
                        timer.reset();
                    }
                    break;
                case SCORING:
                    if (timer.milliseconds() > 100) {
                        samplesScored++;
                        if (samplesScored == 5) {
                            currentState = State.PARK;
                            drive.followTrajectorySequenceAsync(park);
                            setArmPosition(0);
                        } else {
                            currentState = State.RETURN;
                            setArmPosition(-600);
                            if (samplesScored == 4)
                                drive.followTrajectorySequenceAsync(returnToZoneLater);
                            else
                                drive.followTrajectorySequenceAsync(returnToZone);

                        }
                        bumper.setPosition(0);
                    }
                    break;
                case RETURN:
                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_SCORE;
                        if (samplesScored == 2) {
                            drive.followTrajectorySequenceAsync(driveToScoreThird);
                        } else if (samplesScored == 3) {
                            drive.followTrajectorySequenceAsync(driveToScoreFourth);
                        } else if (samplesScored == 4) {
                            drive.followTrajectorySequenceAsync(driveToScoreFifth);
                        }
                        bumper.setPosition(1);
                    } else if (atFlipPosition(FLIP_INTAKE)) {
                        if (samplesScored == 4)
                            setArmPosition(GRAB_POSITION - 50);
                        else if (samplesScored == 3)
                            setArmPosition(GRAB_POSITION - 25);
                        else
                            setArmPosition(GRAB_POSITION);
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;

            }

            drive.update();

        }
        endPose = park.end();

    }

    public void setArmPosition(int position) {
        setArmPosition(position, 1);
    }
    public void setArmPosition(int position, double power) {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
        arm.setTargetPosition(position);
    }

    public void setExtendPosition(int position) {
        setExtendPosition(position, 1);
    }
    public void setExtendPosition(int position, double power) {
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setPower(power);
        extend.setTargetPosition(position);
    }

    public boolean atArmPosition(int target) {
        return Math.abs(target - arm.getCurrentPosition()) < 10;
    }


    public boolean atFlipPosition(int target) {
        return Math.abs(target - flip.getCurrentPosition()) < 10;
    }

}
