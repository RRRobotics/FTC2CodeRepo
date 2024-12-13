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

@Autonomous(name = "RRAuto2")
public class RRAuto2 extends LinearOpMode {

    DcMotor FR, FL, BR, BL;
    DcMotor arm, extend, flip;
    Servo pitch, heading;
    CRServo intake;
    private double speed_factor = 0.4;
    private final int GRAB_POSITION = -219;

    private final int MAX_POSITION = -2092;

    private final int SCORED_POSITION = -1324;
    private final int FLIP_SCORE = 0;
    private final int FLIP_INTAKE = 0;

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
        heading = hardwareMap.get(Servo.class, "heading");
        pitch = hardwareMap.get(Servo.class, "pitch");
        intake = hardwareMap.get(CRServo.class, "Intake");
        telemetry.addData("initialization:", "is a success");
        telemetry.update();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setTargetPosition(0);
        flip.setTargetPosition(0);
        flip.setPower(1);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        endPose = new Pose2d();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence scoreFirst = drive.trajectorySequenceBuilder(new Pose2d(8.50, -66.35, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(8.50, -36.00, Math.toRadians(90.00)))
                .addDisplacementMarker(30, () -> {setArmPosition(SCORED_POSITION);})
                .build();

        drive.setPoseEstimate(scoreFirst.start());

        TrajectorySequence pushOthers = drive.trajectorySequenceBuilder(new Pose2d(8.5, -36, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(33.00, -50.00, Math.toRadians(70.00)))
                .turn(-Math.toRadians(140))
                .lineToLinearHeading(new Pose2d(43.00, -50.00, Math.toRadians(70)))
                .turn(-Math.toRadians(140))
                .lineToLinearHeading(new Pose2d(53.00, -50.00, Math.toRadians(70)))
                .turn(-Math.toRadians(140))
                .splineToLinearHeading(new Pose2d(40.00, -65, Math.toRadians(270)), Math.toRadians(270))
                .build();

        TrajectorySequence driveToScore = drive.trajectorySequenceBuilder(new Pose2d(40.00, -63, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(6.5, -39.00, Math.toRadians(270.00)))
                .build();


        TrajectorySequence returnToZone = drive.trajectorySequenceBuilder(new Pose2d(6.5, -39.00, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(25, -44.52))
                .splineToConstantHeading(new Vector2d(40.00, -63), Math.toRadians(270.00))
                .build();


        waitForStart();

        pitch.setPosition(1);
        heading.setPosition(0.52);

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
                        samplesScored++;
                    }
                    break;
                case PUSH_OTHERS:
                    if (drive.getPoseEstimate().getY() < -45 && drive.getPoseEstimate().getX() > 20) {
                        double heading = drive.getPoseEstimate().getHeading();
//                      ???  heading += Math.PI/2.0;
                        pitch.setPosition(0);
                        this.heading.setPosition(-heading/Math.PI + 2.0);
                        int position = -Math.abs((int)(400.0/Math.cos(heading)));
                        if (position > -2100)
                            setExtendPosition(position);
                        else
                            setExtendPosition(-2100);
                        telemetry.addData("extend", extend.getTargetPosition());
                        telemetry.addData("heading", heading);
                        telemetry.addData("cosine heading", Math.cos(heading));
                        telemetry.update();
                        flip.setTargetPosition(FLIP_INTAKE);
                    }
                    else {
                        setExtendPosition(0);
                        heading.setPosition(0.52);
                        pitch.setPosition(1);
                    }

                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_SCORE;
                        drive.followTrajectorySequenceAsync(driveToScore);
                        setArmPosition(MAX_POSITION);
                        flip.setTargetPosition(FLIP_SCORE);
                    }
                    break;
                case DRIVE_TO_SCORE:
                    if (!drive.isBusy()) {
                        currentState = State.SCORING;
                        setArmPosition(SCORED_POSITION);
                    }
                    break;
                case SCORING:
                    if (atArmPosition(SCORED_POSITION)) {
                        samplesScored++;
                        if (samplesScored == 5) {
//                            currentState = State.PARK;
//                            drive.followTrajectorySequenceAsync(park);
                            currentState = State.IDLE;
                            setArmPosition(0);
                        } else {
                            currentState = State.RETURN;
                            drive.followTrajectorySequenceAsync(returnToZone);
                            setArmPosition(GRAB_POSITION);
                            flip.setTargetPosition(FLIP_INTAKE);
                        }
                    }
                    break;
                case RETURN:
                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_SCORE;
                        drive.followTrajectorySequenceAsync(driveToScore);
                        setArmPosition(MAX_POSITION);
                        flip.setTargetPosition(FLIP_SCORE);
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

}
