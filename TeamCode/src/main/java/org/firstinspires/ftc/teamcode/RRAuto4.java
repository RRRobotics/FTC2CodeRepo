package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.atomic.AtomicBoolean;


@Autonomous(name = "RRAuto4")
public class RRAuto4 extends LinearOpMode {

    DcMotorEx FR, FL, BR, BL, arm, extend, flip;
    Servo pitch1, pitch2, bumper;
    CRServo heading, grabber;
    TouchSensor top;

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
    CustomPID armPID;


    public void runOpMode() {
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        extend = hardwareMap.get(DcMotorEx.class, "extend");
        flip = hardwareMap.get(DcMotorEx.class, "flip");
        pitch1 = hardwareMap.get(Servo.class, "pitch1");
        pitch2 = hardwareMap.get(Servo.class, "pitch2");
        pitch2.setDirection(Servo.Direction.REVERSE);
        heading = hardwareMap.get(CRServo.class, "heading");
        grabber = hardwareMap.get(CRServo.class, "grabber");
        bumper = hardwareMap.get(Servo.class, "bumper");
        bumper.setDirection(Servo.Direction.REVERSE);
        top = hardwareMap.touchSensor.get("top");
        telemetry.addData("initialization:", "is a success");
        telemetry.update();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        endPose = new Pose2d();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence scoreFirst = drive.trajectorySequenceBuilder(new Pose2d(8.50, -66.35, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(8.50, -35.50, Math.toRadians(90.00)))
                .build();

        drive.setPoseEstimate(scoreFirst.start());

        AtomicBoolean raising = new AtomicBoolean(false);


        TrajectorySequence pushOthers = drive.trajectorySequenceBuilder(new Pose2d(20, -45, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36, -28, Math.toRadians(0.00)), Math.toRadians(100.00))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(36.5, -37.00, Math.toRadians(-70.00)))
                .splineToLinearHeading(new Pose2d(46.25, -27, Math.toRadians(0.00)), Math.toRadians(50.00))
                .turn(0.01)
                .lineToLinearHeading(new Pose2d(46.25, -37.00, Math.toRadians(-70.00)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(63, -15.19, Math.toRadians(90.00)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(63, -45), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(50, -65.5), Math.toRadians(270))
                .build();


        TrajectorySequence driveToScoreFar = drive.trajectorySequenceBuilder(new Pose2d(50, -65.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(20, -51, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(4, -33, Math.toRadians(90)), Math.toRadians(90))
                .addDisplacementMarker(56.5, () -> {armPID.setTarget(SCORED_POSITION);})
                .addDisplacementMarker(0.5,  () -> {armPID.setTarget(MAX_POSITION);})
                .build();

        TrajectorySequence driveToScoreThird = drive.trajectorySequenceBuilder(new Pose2d(36, -67, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(20, -60, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(1, -33, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(false)
                .addDisplacementMarker(48.5, () -> {armPID.setTarget(SCORED_POSITION);})
                .addDisplacementMarker(0.5, () -> {armPID.setTarget(MAX_POSITION);})
                .build();

        TrajectorySequence driveToScoreFourth = drive.trajectorySequenceBuilder(new Pose2d(36, -68, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(20, -60, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-5, -32.75, Math.toRadians(90)), Math.toRadians(90))
                .addDisplacementMarker(51.75, () -> {armPID.setTarget(SCORED_POSITION);})
                .addDisplacementMarker(0.5, () -> {armPID.setTarget(MAX_POSITION);})
                .build();

        TrajectorySequence driveToScoreFifth = drive.trajectorySequenceBuilder(new Pose2d(36, -68.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(20, -65, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(  -10, -33.5, Math.toRadians(90)), Math.toRadians(90))
                .addDisplacementMarker(58, () -> {armPID.setTarget(SCORED_POSITION);})
                .addDisplacementMarker(0.5, () -> {armPID.setTarget(MAX_POSITION);})
                .build();

        TrajectorySequence returnToZone = drive.trajectorySequenceBuilder(new Pose2d(4, -33, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(36, -67), Math.toRadians(270.00))
                .setReversed(false)
                .addDisplacementMarker(5, () -> {setFlipPosition(FLIP_INTAKE);})
                .addTemporalMarker(2.15, () -> {armPID.setTarget(SCORED_POSITION);
                    raising.set(true);})
                .build();

        TrajectorySequence returnToZoneLater = drive.trajectorySequenceBuilder(new Pose2d(1, -33, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(36, -68), Math.toRadians(270.00))
                .setReversed(false)
                .addDisplacementMarker(5, () -> {setFlipPosition(FLIP_INTAKE);})
                .addTemporalMarker(2.15, () -> {armPID.setTarget(SCORED_POSITION);
                    raising.set(true);})
                .build();

        TrajectorySequence returnToZoneLast = drive.trajectorySequenceBuilder(new Pose2d(-5, -32.75, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(36, -68.5), Math.toRadians(270.00))
                .setReversed(false)
                .addDisplacementMarker(5, () -> {setFlipPosition(FLIP_INTAKE);})
                .addTemporalMarker(2.15, () -> {armPID.setTarget(SCORED_POSITION);
                    raising.set(true);})
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-10, -33.5, Math.toRadians(90.00)))
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(40.00, -68))
                .setReversed(false)
                .build();


        waitForStart();

        extend.setTargetPosition(0);

        setFlipPosition(FLIP_SCORE);

        armPID = new CustomPID(arm, 0.0004, 0.0, 0.00002, 0);

        bumper.setPosition(1);
        pitch1.setPosition(0);
        pitch2.setPosition(0);

        if(isStopRequested()) return;

        drive.followTrajectorySequenceAsync(scoreFirst);
        currentState = State.SCORE_FIRST;

        armPID.setTarget(MAX_POSITION);

        ElapsedTime timer = new ElapsedTime();

        int samplesScored = 0;
        double robotHeading, servoHeading;
        boolean intaking;

        raising.set(false);

        boolean flipped = false;

        ElapsedTime flipTimer = new ElapsedTime();

        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("arm current", arm.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("extend current", extend.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("flip current", flip.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("flip", flip.getCurrentPosition());
            telemetry.addData("flip target", flip.getTargetPosition());
            System.out.println(flip.getTargetPosition());
            System.out.println(currentState);
            telemetry.update();
            switch(currentState) {
                case SCORE_FIRST:
                    if (!drive.isBusy()) {
                        currentState = State.SCORING_FIRST;
                        armPID.setTarget(SCORED_POSITION);
                        timer.reset();
                    } else if (armPID.getPosition() < -5000) {
                        setFlipPosition(FLIP_SCORE);
                    }
                    break;
                case SCORING_FIRST:
                    if (timer.milliseconds() > 100) {
                        currentState = State.PUSH_OTHERS;
                        drive.followTrajectorySequenceAsync(pushOthers);
                        armPID.setTarget(GRAB_POSITION);
                        samplesScored++;
                        bumper.setPosition(0);
                        setFlipPosition(FLIP_SCORE / 2);
                        timer.reset();
                    }
                    break;
                case PUSH_OTHERS:
                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_SCORE;
                        drive.followTrajectorySequenceAsync(driveToScoreFar);
                        bumper.setPosition(1);
                        extend.setPower(0.1);
                    }
                    double diff = 2.5;
                    if (timer.seconds() < 3.2 + diff) {
                        setFlipPosition(FLIP_SCORE / 2);
                    }
                    if (timer.seconds() > 3.2 + diff) {
                        grabber.setPower(0);
                        setFlipPosition(FLIP_INTAKE);
                    } else if (timer.seconds() > 3 + diff) {
                        setExtendPosition(0);
                        pitch1.setPosition(0.1);
                        pitch2.setPosition(0.1);
                    } else if (timer.seconds() > 2.7 + diff) {
                        grabber.setPower(-1);
                    }  else if (timer.seconds() > 2.1 + diff) {
                        setExtendPosition(MAX_EXTEND);
                    } else if (timer.seconds() > 2 + diff) {
                        pitch1.setPosition(0.5);
                        pitch2.setPosition(0.5);
                    } else if (timer.seconds() > 1.4 + diff) {
                        pitch1.setPosition(1);
                        pitch2.setPosition(1);
                        grabber.setPower(1);
                    } else if (timer.seconds() > 1 + diff) {
                        grabber.setPower(0);
                    }
                    else if (timer.seconds() > 0.4 + diff) {
                        setExtendPosition(-500);
                    }
                    else if (timer.seconds() > 2.7) {
                        grabber.setPower(-1);
                    } else if (timer.seconds() > 2) {
                        setExtendPosition(MAX_EXTEND);
                    } else if (timer.seconds() > 1.9) {
                        pitch1.setPosition(0.6);
                        pitch2.setPosition(0.6);
                    } else if (timer.seconds() > 1.6) {
                        grabber.setPower(1);
                    } else if (timer.seconds() > 0.85) {
                        setExtendPosition(-500);
                        pitch1.setPosition(1);
                        pitch2.setPosition(1);
                    }
                    break;
                case DRIVE_TO_SCORE:
                    if (!drive.isBusy()) {
                        currentState = State.SCORING;
                        armPID.setTarget(SCORED_POSITION);
                        timer.reset();
                    } if (armPID.getPosition() < MID_POSITION / 2) {
                        setFlipPosition(FLIP_SCORE);
                        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        setExtendPosition(0);
                    }
                    break;
                case SCORING:
                    if (timer.milliseconds() > 20) {
                        samplesScored++;
                        if (samplesScored == 5) {
                            currentState = State.PARK;
                            drive.followTrajectorySequenceAsync(park);
                            armPID.setTarget(0);
                        } else {
                            currentState = State.RETURN;
                            raising.set(false);
                            armPID.setTarget(MID_POSITION);
                            setFlipPosition(FLIP_INTAKE);
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
                        if (samplesScored < 5)
                            currentState = State.DRIVE_TO_SCORE;
                        else {
                            currentState = State.PARK;
                            armPID.setTarget(0);
                        }
                        if (samplesScored == 2) {
                            drive.followTrajectorySequenceAsync(driveToScoreThird);
                        } else if (samplesScored == 3) {
                            drive.followTrajectorySequenceAsync(driveToScoreFourth);
                        } else if (samplesScored == 4) {
                            drive.followTrajectorySequenceAsync(driveToScoreFifth);
                        }
                        bumper.setPosition(1);
                    } else if (atFlipPosition(FLIP_INTAKE) && !raising.get() && samplesScored < 5) {
                        armPID.setTarget(GRAB_POSITION);
                    }

                    if (flip.getCurrentPosition() > -10 && flip.getTargetPosition() == FLIP_INTAKE) {
                        flip.setPower(0.2);
                    }

                    break;
                case PARK:
                    setFlipPosition(FLIP_INTAKE);
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }
//
//            if (flip.getTargetPosition() == FLIP_INTAKE && flip.getCurrentPosition() > -5 && !top.isPressed()) {
//                flipped = false;
//                flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                flip.setPower(1);
//            } else if (flip.getTargetPosition() == FLIP_INTAKE && top.isPressed() && !flipped) {
//                flipTimer.reset();
//                flipped = true;
//            } else if (flip.getTargetPosition() == FLIP_INTAKE && top.isPressed() && flipTimer.seconds() > 1) {
//                flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                setFlipPosition(0);
//            }

            drive.update();
            armPID.update();

        }
        endPose = park.end();

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

    public void setFlipPosition(int position) {
        setFlipPosition(position, 0.3);
    }

    public void setFlipPosition(int position, double power) {
        flip.setPower(power);
        flip.setTargetPosition(position + flipOffset);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
