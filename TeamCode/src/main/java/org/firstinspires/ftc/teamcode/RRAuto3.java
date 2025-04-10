package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.*;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.atomic.AtomicBoolean;


@Autonomous(name = "RRAuto3")
@Disabled
public class RRAuto3 extends LinearOpMode {

    DcMotor FR, FL, BR, BL;
    DcMotor arm, extend, flip;
    Servo pitch1, pitch2, bumper;
    CRServo heading, grabber;

    private double speed_factor = 0.6;
    private int offset, flipOffset;
    private final int GRAB_POSITION = DriveConstants.getGRAB_POSITION();
    private final int MAX_POSITION = DriveConstants.getMAX_POSITION();
    private final int SCORED_POSITION = DriveConstants.getSCORED_POSITION();
    private final int MAX_EXTEND = DriveConstants.getMAX_EXTEND();
    private final int FLIP_SCORE = DriveConstants.getFLIP_SCORE();
    private final int FLIP_INTAKE = DriveConstants.getFLIP_INTAKE();
    private final int MID_POSITION = DriveConstants.getMID_POSITION();
    private RRTeleOpFieldOriented.ArmState armState = RRTeleOpFieldOriented.ArmState.X;

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
        pitch1 = hardwareMap.get(Servo.class, "pitch1");
        pitch2 = hardwareMap.get(Servo.class, "pitch2");
        pitch2.setDirection(Servo.Direction.REVERSE);
        heading = hardwareMap.get(CRServo.class, "heading");
        grabber = hardwareMap.get(CRServo.class, "grabber");
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
//                .addDisplacementMarker(30.7, () -> {setArmPosition(SCORED_POSITION);})
                .build();

        drive.setPoseEstimate(scoreFirst.start());

        AtomicBoolean raising = new AtomicBoolean(false);


        TrajectorySequence pushOthers = drive.trajectorySequenceBuilder(new Pose2d(8.5, -35.4, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(38, -32, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(37, -18, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(46, -18, Math.toRadians(90)), Math.toRadians(270.00))
                .splineToLinearHeading(new Pose2d(42, -47, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(37, -18, Math.toRadians(90)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(53, -18, Math.toRadians(90)), Math.toRadians(270.00))
                .splineToLinearHeading(new Pose2d(50, -48, Math.toRadians(90)), Math.toRadians(270.00))
                .splineToLinearHeading(new Pose2d(44, -15, Math.toRadians(90)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(59, -15, Math.toRadians(90)), Math.toRadians(270.00))
                .splineToLinearHeading(new Pose2d(61, -20, Math.toRadians(90)), Math.toRadians(270.00))
                .splineToLinearHeading(new Pose2d(61, -45, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(61, -50, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(40, -68, Math.toRadians(90)), Math.toRadians(270))
                .setReversed(false)
                .addTemporalMarker(9, () -> {setArmPosition(MAX_POSITION);})
                .build();

        TrajectorySequence driveToScoreFar = drive.trajectorySequenceBuilder(new Pose2d(40.00, -68, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(10, -60, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-7, -35.4, Math.toRadians(90)), Math.toRadians(90)).addDisplacementMarker(70, () -> {setArmPosition(SCORED_POSITION);})
                .addDisplacementMarker(5, () -> {flip.setTargetPosition(FLIP_SCORE);})
                .addDisplacementMarker(0.5, () -> {setArmPosition(MAX_POSITION);})
                .build();

        TrajectorySequence driveToScoreThird = drive.trajectorySequenceBuilder(new Pose2d(34.00, -69.3, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(10, -60, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-6, -35.7, Math.toRadians(90)), Math.toRadians(90)).setReversed(false)

                .addDisplacementMarker(5, () -> {flip.setTargetPosition(FLIP_SCORE);})
                .addDisplacementMarker(0.5, () -> {setArmPosition(MAX_POSITION);})
                .build();

        TrajectorySequence driveToScoreFourth = drive.trajectorySequenceBuilder(new Pose2d(34.00, -72, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(10, -60, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-10, -35.7, Math.toRadians(90)), Math.toRadians(90))

                .addDisplacementMarker(5, () -> {flip.setTargetPosition(FLIP_SCORE);})
                .addDisplacementMarker(0.5, () -> {setArmPosition(MAX_POSITION);})
                .build();

        TrajectorySequence driveToScoreFifth = drive.trajectorySequenceBuilder(new Pose2d(34.00, -72, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(10, -60, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-14, -35.7, Math.toRadians(90)), Math.toRadians(90))
//                .addDisplacementMarker(52, () -> {setArmPosition(SCORED_POSITION);})
                .addDisplacementMarker(5, () -> {flip.setTargetPosition(FLIP_SCORE);})
                .addDisplacementMarker(0.5, () -> {setArmPosition(MAX_POSITION);})
                .build();

        TrajectorySequence returnToZone = drive.trajectorySequenceBuilder(new Pose2d(-4, -35.5, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(34, -69.3), Math.toRadians(270.00))
                .setReversed(false)
                .addDisplacementMarker(5, () -> {flip.setTargetPosition(FLIP_INTAKE);})
                .addTemporalMarker(2.15, () -> {setArmPosition(SCORED_POSITION);
                    raising.set(true);})
                .build();

        TrajectorySequence returnToZoneLater = drive.trajectorySequenceBuilder(new Pose2d(-6, -35.5, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(34, -72), Math.toRadians(270.00))
                .setReversed(false)
                .addDisplacementMarker(5, () -> {flip.setTargetPosition(FLIP_INTAKE);})
                .addTemporalMarker(2.15, () -> {setArmPosition(SCORED_POSITION);
                    raising.set(true);})
                .build();

        TrajectorySequence returnToZoneLast = drive.trajectorySequenceBuilder(new Pose2d(-10, -35.5, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(34, -72), Math.toRadians(270.00))
                .setReversed(false)
                .addDisplacementMarker(5, () -> {flip.setTargetPosition(FLIP_INTAKE);})
                .addTemporalMarker(2.15, () -> {setArmPosition(SCORED_POSITION);
                    raising.set(true);})
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-8, -35.5, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(40.00, -68), Math.toRadians(270.00))
                .setReversed(false)
                .build();


        waitForStart();

        extend.setTargetPosition(0);
        flip.setTargetPosition(0);
        flip.setPower(1);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setTargetPosition(0);
        bumper.setPosition(1);
        pitch1.setPosition(0);
        pitch2.setPosition(0);

        if(isStopRequested()) return;

        drive.followTrajectorySequenceAsync(scoreFirst);
        currentState = State.SCORE_FIRST;

        setArmPosition(MAX_POSITION);

        ElapsedTime timer = new ElapsedTime();

        int samplesScored = 0;
        double robotHeading, servoHeading;
        boolean intaking;

        flip.setTargetPosition(0);

        raising.set(false);

        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("flip", flip.getCurrentPosition());
            telemetry.addData("flip target", flip.getTargetPosition());
            telemetry.update();
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
                        setArmPosition(MID_POSITION);
                        samplesScored++;
                        bumper.setPosition(0);
                        timer.reset();
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
                    if (timer.seconds() > 1) {
                        flip.setTargetPosition(FLIP_INTAKE);
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
                            raising.set(false);
                            setArmPosition(MID_POSITION);
                            if (samplesScored == 3)
                                drive.followTrajectorySequenceAsync(returnToZoneLater);
                            else if (samplesScored == 4) {
                                drive.followTrajectorySequenceAsync(returnToZoneLast);
                            } else
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
                    } else if (atFlipPosition(FLIP_INTAKE) && !raising.get()) {
                        //if (samplesScored == 4)
                          //  setArmPosition(GRAB_POSITION - 50);
                        //else if (samplesScored == 3)
                          //  setArmPosition(GRAB_POSITION - 25);
                        //else
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
        return Math.abs(target - flip.getCurrentPosition()) < 20;
    }

}
