package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.MAX_POSITION;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    DcMotor Arm;
    Servo Extend;
    Servo Pivot;
    CRServo Intake;
    private double speed_factor = 0.4;
    private final int GRAB_POSITION = -1200;
    private final int MAX_POSITION = -5700;
    private final int SCORED_POSITION = -4000;

    enum State {
        IDLE, SCORE_FIRST, SCORING_FIRST, PUSH_OTHERS, SCORING, DRIVE_TO_SCORE, RETURN
    }

    State currentState = State.IDLE;



    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Extend = hardwareMap.get(Servo.class, "Extend");
        Pivot = hardwareMap.get(Servo.class, "Pivot");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        telemetry.addData("initialization:", "is a success");
        telemetry.update();
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence scoreFirst = drive.trajectorySequenceBuilder(new Pose2d(8.50, -66.35, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(8.52, -34.00, Math.toRadians(90.00)))
                .build();


        drive.setPoseEstimate(scoreFirst.start());

        TrajectorySequence pushOthers = drive.trajectorySequenceBuilder(new Pose2d(8.50, -33.00, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(45.99, -9.39), Math.toRadians(90.00))
                .setReversed(false)
                .lineTo(new Vector2d(46.23, -56.59))
                .splineToConstantHeading(new Vector2d(51.29, -11.56), Math.toRadians(59.89))
                .splineToConstantHeading(new Vector2d(56.35, -19.75), Math.toRadians(-68.89))
                .splineToConstantHeading(new Vector2d(56.11, -56.59), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(57.55, -14.93), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(62.38, -13.70), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(61.88, -56.23), Math.toRadians(90.00))
                .splineTo(new Vector2d(51.77, -55.34), Math.toRadians(205.95))
                .splineTo(new Vector2d(49.68, -62.70), Math.toRadians(270.00))
                .build();



        TrajectorySequence driveToScore = drive.trajectorySequenceBuilder(new Pose2d(49.68, -62.70, Math.toRadians(270.00)))
                .lineToSplineHeading(new Pose2d(3.18, -32.99, Math.toRadians(90.00)))
                .build();

        TrajectorySequence returnToZone = drive.trajectorySequenceBuilder(new Pose2d(3.18, -33.00, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(49.68, -62.70, Math.toRadians(270.00)), Math.toRadians(270.00))
                .setReversed(false)
                .build();





        ElapsedTime waitTimer = new ElapsedTime();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequenceAsync(scoreFirst);
        currentState = State.SCORE_FIRST;

        setArmPosition(MAX_POSITION);
        
        sleep(1000);
        
        int count = 0;
        
        while(opModeIsActive() && !isStopRequested()) {


            switch(currentState) {
                case SCORE_FIRST:
                    if (!drive.isBusy()) {
                        currentState = State.SCORING_FIRST;
                        waitTimer.reset();
                        setArmPosition(SCORED_POSITION);
                    }
                    break;
                case SCORING_FIRST:
                    if (waitTimer.seconds() > 1) {
                        currentState = State.PUSH_OTHERS;
                        drive.followTrajectorySequenceAsync(pushOthers);
                        setArmPosition(GRAB_POSITION);
                    }
                    break;
                case PUSH_OTHERS:
                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_SCORE;
                        drive.followTrajectorySequenceAsync(driveToScore);
                    }
                    break;
                case DRIVE_TO_SCORE:
                    if (!drive.isBusy()) {
                        currentState = State.SCORING;
                        setArmPosition(SCORED_POSITION);
                        waitTimer.reset();
                    }
                    break;
                case SCORING:
                    if (waitTimer.seconds() > 1) {
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
                case RETURN:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            drive.update();

        }

    }

    public void setArmPosition(int position) {
        setArmPosition(position, 1);
    }
    public void setArmPosition(int position, double power) {
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(power);
        Arm.setTargetPosition(position);
    }

}
