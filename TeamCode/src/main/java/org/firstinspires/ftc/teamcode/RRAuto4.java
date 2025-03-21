package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    TouchSensor bottom, top;

    private final int GRAB_POSITION = DriveConstants.getGRAB_POSITION();
    private final int MAX_POSITION = DriveConstants.getMAX_POSITION();
    private final int SCORED_POSITION = DriveConstants.getSCORED_POSITION();
    private final int MAX_EXTEND = DriveConstants.getMAX_EXTEND();
    private final int FLIP_SCORE = DriveConstants.getFLIP_SCORE();
    private final int FLIP_INTAKE = DriveConstants.getFLIP_INTAKE();
    private final int MID_POSITION = DriveConstants.getMID_POSITION();
//    private RRTeleOpFieldOriented.ArmState armState = RRTeleOpFieldOriented.ArmState.X;


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
        bottom = hardwareMap.touchSensor.get("bottom");
        top = hardwareMap.touchSensor.get("top");
        telemetry.addData("initialization:", "is a success");
        telemetry.update();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence scoreFirst = drive.trajectorySequenceBuilder(new Pose2d(8.50, -66.35, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(9, -35.5, Math.toRadians(90.00)))
                .addDisplacementMarker(30.7, () -> {armPID.setTarget(SCORED_POSITION);})
                .build();

        drive.setPoseEstimate(scoreFirst.start());

        AtomicBoolean raising = new AtomicBoolean(false);

        TrajectorySequence pushOthers = drive.trajectorySequenceBuilder(new Pose2d(9, -37, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(27, -39, Math.toRadians(24.00)), Math.toRadians(100.00))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(36.5, -43.00, Math.toRadians(-70.00 )))
                .splineToLinearHeading(new Pose2d(46.75, -27.5, Math.toRadians(0.00)), Math.toRadians(50.00))
                .turn(0.01)
                .lineToLinearHeading(new Pose2d(46.75, -38, Math.toRadians(-70.00)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(63, -15, Math.toRadians(90.00)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(63, -45), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(50, -69), Math.toRadians(270))
                .build();


        TrajectorySequence driveToScoreFar = drive.trajectorySequenceBuilder(new Pose2d(50, -68, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(20, -51, Math.toRadians(90.00)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 10))
                .splineToLinearHeading(new Pose2d(7, -36, Math.toRadians(90)), Math.toRadians(90))
//                .addDisplacementMarker(56.5, () -> {armPID.setTarget(SCORED_POSITION);})
//                .addDisplacementMarker(0.5,  () -> {armPID.setTarget(MAX_POSITION);})
                .build();

        TrajectorySequence driveToScoreThird = drive.trajectorySequenceBuilder(new Pose2d(38, -69, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(20, -60, Math.toRadians(90.00)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 15))
                .splineToLinearHeading(new Pose2d(5, -36, Math.toRadians(90)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 15))
//                .addDisplacementMarker(49, () -> {armPID.setTarget(SCORED_POSITION);})
//                .addDisplacementMarker(0.5, () -> {armPID.setTarget(MAX_POSITION);})
                .build();

        TrajectorySequence driveToScoreFourth = drive.trajectorySequenceBuilder(new Pose2d(38, -69, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(20, -60, Math.toRadians(90.00)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 15))
                .splineToLinearHeading(new Pose2d(3, -36, Math.toRadians(90)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 15))
//                .addDisplacementMarker(52, () -> {armPID.setTarget(SCORED_POSITION);})
//                .addDisplacementMarker(0.5, () -> {armPID.setTarget(MAX_POSITION);})
                .build();

        TrajectorySequence driveToScoreFifth = drive.trajectorySequenceBuilder(new Pose2d(38, -69, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(20, -60, Math.toRadians(90.00)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 15))
                .splineToLinearHeading(new Pose2d(1, -36, Math.toRadians(90)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 15))
//                .addDisplacementMarker(55.5, () -> {armPID.setTarget(SCORED_POSITION);})
//                .addDisplacementMarker(0.5, () -> {armPID.setTarget(MAX_POSITION);})
                .build();

        TrajectorySequence returnToZone = drive.trajectorySequenceBuilder(new Pose2d(7, -37, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(38, -67), Math.toRadians(270.00),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 15))
                .setReversed(false)
                .addDisplacementMarker(5, () -> {setFlipPosition(FLIP_INTAKE);})
                .addTemporalMarker(2.15, () -> {
//                    armPID.setTarget(MAX_POSITION);
                    raising.set(true);})
                .build();

        TrajectorySequence returnToZoneLater = drive.trajectorySequenceBuilder(new Pose2d(5, -37, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(38, -67), Math.toRadians(270.00),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 15))
                .setReversed(false)
                .addDisplacementMarker(5, () -> {setFlipPosition(FLIP_INTAKE);})
                .addTemporalMarker(2.15, () -> {
//                    armPID.setTarget(MAX_POSITION);
                    raising.set(true);})
                .build();

        TrajectorySequence returnToZoneLast = drive.trajectorySequenceBuilder(new Pose2d(3, -37, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(38, -67), Math.toRadians(270.00),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 15))
                .setReversed(false)
                .addDisplacementMarker(5, () -> {setFlipPosition(FLIP_INTAKE);})
                .addTemporalMarker(2.15, () -> {
//                    armPID.setTarget(MAX_POSITION);
                    raising.set(true);})
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(1, -37, Math.toRadians(90.00)))
                .addTemporalMarker(0.5, () -> {setFlipPosition(FLIP_INTAKE);})
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(36, -60))
                .setReversed(false)
                .build();

        armPID = new CustomPID(arm, 0.0003, 0.0, 0.0000, 0);

        int samplesScored = 0;
        double robotHeading, servoHeading;
        boolean intaking;

        raising.set(false);

        boolean flipped = false;
        boolean lowered = false;

        boolean push = false;
        boolean grab = false;

        pitch1.setPosition(0);
        pitch2.setPosition(0);
        bumper.setPosition(0);

        waitForStart();

        bumper.setPosition(1);

        extend.setTargetPosition(0);

        setFlipPosition(FLIP_SCORE);


        if(isStopRequested()) return;

        drive.followTrajectorySequenceAsync(scoreFirst);
        currentState = State.SCORE_FIRST;

        armPID.setTarget(MAX_POSITION);

        ElapsedTime timer = new ElapsedTime();

        while(opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("arm current", arm.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("extend current", extend.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("flip current", flip.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("flip", flip.getCurrentPosition());
//            telemetry.addData("flip target", flip.getTargetPosition());
//            telemetry.update();
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
                        armPID.setTarget(MID_POSITION);
                        samplesScored++;
                        bumper.setPosition(0);
                        setFlipPosition(FLIP_SCORE / 2, 0.7);
                        timer.reset();
                    }
                    break;
                case PUSH_OTHERS:
                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_SCORE;
                        drive.followTrajectorySequenceAsync(driveToScoreFar);
                        bumper.setPosition(1);
                        extend.setPower(0.1);
                        armPID.setTarget(MAX_POSITION);
//                        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), -67, Math.toRadians(90)));
                    }
                    if (atFlipPosition(FLIP_SCORE / 2)) {
                        armPID.setTarget(GRAB_POSITION);
                    }
                    double diff = 2.55;
                    if (timer.seconds() < 3.2 + diff) {
                        setFlipPosition(FLIP_SCORE / 2);
                    }
                    if (timer.seconds() > 4 + diff) {
                        setFlipPosition(20);
                    } else if (timer.seconds() > 3.2 + diff) {
                        grabber.setPower(0);
                        setFlipPosition(0, 0.5);
                    } else if (timer.seconds() > 3 + diff) {
                        setExtendPosition(0);
                        pitch1.setPosition(0.1);
                        pitch2.setPosition(0.1);
                    } else if (timer.seconds() > 2.65 + diff) {
                        grabber.setPower(-1);
                    }  else if (timer.seconds() > 2.1 + diff) {
                        setExtendPosition(MAX_EXTEND);
                    } else if (timer.seconds() > 2 + diff) {
                        pitch1.setPosition(0.5);
                        pitch2.setPosition(0.5);
                    } else if (timer.seconds() > 1.35 + diff) {
                        pitch1.setPosition(1);
                        pitch2.setPosition(1);
                        grabber.setPower(1);
                    } else if (timer.seconds() > 1 + diff) {
                        grabber.setPower(0);
                    }
                    else if (timer.seconds() > 0.4 + diff) {
                        setExtendPosition(-500);
                        pitch1.setPosition(0.4);
                        pitch2.setPosition(0.4);
                    }
                    else if (timer.seconds() > 2.6) {
                        grabber.setPower(-1);
                    } else if (timer.seconds() > 2) {
                        setExtendPosition(-1500);
                    } else if (timer.seconds() > 1.9) {
                        pitch1.setPosition(0.6);
                        pitch2.setPosition(0.6);
                    } else if (timer.seconds() > 1.4) {
                        grabber.setPower(1);
                    } else if (timer.seconds() > 0.8) {
                        pitch1.setPosition(1);
                        pitch2.setPosition(1);
                    } else if (timer.seconds() > 0.6) {
                        setExtendPosition(-1700);
                    }
                    break;
                case DRIVE_TO_SCORE:
                    if (!drive.isBusy() || (armPID.getTarget() == SCORED_POSITION && armPID.getPosition() > -21000) && drive.getPoseEstimate().getY() > -40) {
                        currentState = State.SCORING;
                        armPID.setTarget(SCORED_POSITION);
                        timer.reset();
                        grab = false;
                        if (samplesScored == 4) {
                            setFlipPosition(FLIP_INTAKE, 1);
                            armPID.setTarget(MID_POSITION);
                        }
                    } if (armPID.getPosition() < MID_POSITION / 2) {
                        setFlipPosition(FLIP_SCORE);
                        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        setExtendPosition(0);
                    } if (drive.getPoseEstimate().getY() > -40 && !push) {
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineTo(new Vector2d(drive.getPoseEstimate().getX(), -34))
                                .addTemporalMarker(0.5, () -> {armPID.setTarget(SCORED_POSITION);})
                                .build());
                        push = true;
                    }
                    break;
                case SCORING:
                    if (timer.milliseconds() > 50) {
                        drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), -37, Math.toRadians(90)));
                        samplesScored++;
                        lowered = false;
                        if (samplesScored == 5) {
                            currentState = State.IDLE;
                            bumper.setPosition(0);
//                            drive.followTrajectorySequenceAsync(park);
                        } else {
                            armPID.setTarget(MID_POSITION);
                            currentState = State.RETURN;
                            raising.set(false);
                            if (samplesScored == 3)
                                drive.followTrajectorySequenceAsync(returnToZoneLater);
                            else if (samplesScored == 4) {
                                drive.followTrajectorySequenceAsync(returnToZoneLast);
                            } else
                                drive.followTrajectorySequenceAsync(returnToZone);

                        }
                        bumper.setPosition(0);
                        setFlipPosition(FLIP_INTAKE);
                    }
                    break;
                case RETURN:
                    if (!drive.isBusy()) {
                        push = false;
                        if (samplesScored < 5) {
//                            flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            currentState = State.DRIVE_TO_SCORE;
                            armPID.setTarget(MAX_POSITION);
                        }
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
                        drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(90)));
                    } else if (atFlipPosition(FLIP_INTAKE) && !raising.get() && samplesScored < 5) {
                        if (!bottom.isPressed() && !lowered) {
                            armPID.setPower(0.8);
                        } else if (bottom.isPressed() && !lowered) {
                            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            armPID.setTarget(0);
                            lowered = true;
                        } else if (lowered) {
                            armPID.setTarget(GRAB_POSITION);
                        }
                    }

                    if (drive.getPoseEstimate().getX() > 30 && drive.isBusy()) {
//                        flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                        flip.setPower(0.5);
                        setFlipPosition(20, 0.7);
                    }
                    if (drive.getPoseEstimate().getX() > 36 && !grab) {
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineTo(new Vector2d(drive.getPoseEstimate().getX(), -73))
                            .build());
                        grab = true;
                    }

                    break;
                case PARK:
                    setFlipPosition(FLIP_INTAKE);
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    } else if (flip.getCurrentPosition() > -20) {
                        armPID.setTarget(0);
                    }
                    break;
                case IDLE:
                    if (flip.getCurrentPosition() > FLIP_SCORE / 2)
                        armPID.setTarget(0);
                    break;
            }

            drive.update();
            armPID.update();

        }

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
        flip.setTargetPosition(position);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
