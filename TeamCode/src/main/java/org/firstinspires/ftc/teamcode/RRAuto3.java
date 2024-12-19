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

import net.jafama.FastMath;

import java.text.DecimalFormat;


@Autonomous(name = "RRAuto3")
public class RRAuto3 extends LinearOpMode {

    DcMotor FR, FL, BR, BL;
    DcMotor arm, extend, flip;
    Servo pitch, heading;
    CRServo intake;
    private double speed_factor = 0.4;
    private final int GRAB_POSITION = -219;

    private final int MAX_POSITION = -2092;

    private final int SCORED_POSITION = -1324;

    //opposite of teleop
    private final int FLIP_SCORE = 0;
    private final int FLIP_INTAKE = 1350;

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
        telemetry.addData("initialization:", "is a success");
        telemetry.update();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        endPose = new Pose2d();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence scoreFirst = drive.trajectorySequenceBuilder(new Pose2d(8.50, -66.35, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(8.50, -36.00, Math.toRadians(90.00)))
                .addDisplacementMarker(29, () -> {setArmPosition(SCORED_POSITION);})
                .build();

        drive.setPoseEstimate(scoreFirst.start());


        TrajectorySequence pushOthers = drive.trajectorySequenceBuilder(new Pose2d(8.5, -36, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(38, -28.96, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(38, -18, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(49.00, -18), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(49.00, -50), Math.toRadians(-88.30))
                .splineToConstantHeading(new Vector2d(40, -18), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(62, -18), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(62, -50), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(52, -18), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(68, -18), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(64, -63), Math.toRadians(270.00))
                .setReversed(false)
                .build();

        TrajectorySequence driveToScoreFar = drive.trajectorySequenceBuilder(new Pose2d(65.00, -64, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(6.5, -39.00, Math.toRadians(90.00)))
                .addDisplacementMarker(63, () -> {setArmPosition(SCORED_POSITION);})
                .build();

        TrajectorySequence driveToScoreClose = drive.trajectorySequenceBuilder(new Pose2d(40.00, -64, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(6.5, -39.00, Math.toRadians(90.00)))
                .addDisplacementMarker(40, () -> {setArmPosition(SCORED_POSITION);})
                .build();


        TrajectorySequence returnToZone = drive.trajectorySequenceBuilder(new Pose2d(6.5, -39.00, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(25, -44.52))
                .splineToConstantHeading(new Vector2d(40.00, -64), Math.toRadians(270.00))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(6.5, -39.00, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(45.00, -60.00))
                .build();


        waitForStart();

        heading.setPosition(0.52);
        extend.setTargetPosition(0);
        flip.setTargetPosition(0);
        flip.setPower(1);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(0);

        if(isStopRequested()) return;

        drive.followTrajectorySequenceAsync(scoreFirst);
        currentState = State.SCORE_FIRST;

        setArmPosition(MAX_POSITION);

        int samplesScored = 0;
        // arm distance constant
        int d = 1450;
        // arm horizontal offset constant
        int h = 440;
        // arm vertical offset constant
        int k = 400;
        double robotHeading, servoHeading;
        boolean intaking;


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
                        setArmPosition(-600);
                        flip.setTargetPosition(FLIP_INTAKE);
                        samplesScored++;
                    }
                    break;
                case PUSH_OTHERS:
                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_SCORE;
                        drive.followTrajectorySequenceAsync(driveToScoreFar);
                        setArmPosition(MAX_POSITION);
                    } else if (atFlipPosition(FLIP_INTAKE)) {
                        setArmPosition(GRAB_POSITION);
                    }
                    break;
                case DRIVE_TO_SCORE:
                    if (!drive.isBusy()) {
                        currentState = State.SCORING;
                        setArmPosition(SCORED_POSITION);
                    }
                    if (arm.getCurrentPosition() < -600)
                        flip.setTargetPosition(FLIP_SCORE);
                    break;
                case SCORING:
                    if (atArmPosition(SCORED_POSITION)) {
                        samplesScored++;
                        if (samplesScored == 5) {
                            currentState = State.PARK;
                            drive.followTrajectorySequenceAsync(park);
                            setArmPosition(0);
                        } else {
                            currentState = State.RETURN;
                            setArmPosition(-600);
                            flip.setTargetPosition(FLIP_INTAKE);
                            drive.followTrajectorySequenceAsync(returnToZone);
                        }
                    }
                    break;
                case RETURN:
                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_SCORE;
                        drive.followTrajectorySequenceAsync(driveToScoreClose);
                        setArmPosition(MAX_POSITION);
                    } else if (atFlipPosition(FLIP_INTAKE)) {
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
