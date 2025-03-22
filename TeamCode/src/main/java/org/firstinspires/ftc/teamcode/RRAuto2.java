package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.MAX_POSITION;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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


@Autonomous(name = "RRAuto2")
@Disabled
public class RRAuto2 extends LinearOpMode {

    DcMotor FR, FL, BR, BL;
    DcMotor arm, extend, flip;
    Servo pitch, heading;
    CRServo intake;
    private double speed_factor = 0.4;
    private final int GRAB_POSITION = -219;

    private final int MAX_POSITION = -2092;

    private final int SCORED_POSITION = -1324;
    private final int FLIP_SCORE = 1350;
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


        int y = -48;
        double turn = Math.toRadians(120);

        TrajectorySequence pushOthers = drive.trajectorySequenceBuilder(new Pose2d(8.5, -36, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(38.00, y, turn / 2.0))
                .lineToLinearHeading(new Pose2d(38.00, y - 10, -turn / 2.0))
                .lineToLinearHeading(new Pose2d(48.00, y, turn / 2.0))
                .lineToLinearHeading(new Pose2d(48.00, y - 10, -turn / 2.0))
                .lineToLinearHeading(new Pose2d(58.00, y, turn / 2.0))
                .lineToSplineHeading(new Pose2d(58.00, -65, Math.toRadians(270)))
                .build();

        TrajectorySequence driveToScore = drive.trajectorySequenceBuilder(new Pose2d(40.00, -63, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(6.5, -39.00, Math.toRadians(270.00)))
                .addDisplacementMarker(40, () -> {setArmPosition(SCORED_POSITION);})
                .build();


        TrajectorySequence returnToZone = drive.trajectorySequenceBuilder(new Pose2d(6.5, -39.00, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(25, -44.52))
                .splineToConstantHeading(new Vector2d(40.00, -63), Math.toRadians(270.00))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(6.5, -39.00, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(45.00, -55.00))
                .build();


        waitForStart();

        pitch.setPosition(0);
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

        // precalculate arm values
        double range = turn + Math.toRadians(10);
        int valuesPerRadian = 100;
        float start = Math.round(-range * 50) / (float)100.0;
        //arm positions
        int[] table = new int[(int)(range * valuesPerRadian)+1];
        for (float theta = start; theta < start + range; theta+= 0.01F) {
            int index = (int)((theta - start) * 100);
            double sin = FastMath.sin(theta);
            double cos = FastMath.cos(theta);
            int position = -(int)((d / cos) // c
                    + (h / (sin * cos)) // a
                    - (h / (sin / cos)) // e
                    - d + k);
            table[index] = position;
            System.out.println("theta: " + theta + " index: " + index + " position: " + position);
        }

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
                    extend.setPower(1);
                    robotHeading = drive.getPoseEstimate().getHeading();
                    if (robotHeading > Math.PI)
                        robotHeading -= 2*Math.PI;
                    intaking = (drive.getPoseEstimate().getY() < y + 5 && drive.getPoseEstimate().getY() > y - 2 && (robotHeading > Math.toRadians(285) || robotHeading < Math.toRadians(90)) && drive.getPoseEstimate().getX() > 15);
                    if (intaking) {
                        // d = center robot to head (> ~|-1500|)
                        // h = center robot to arm axis (440)
                        // a = (d/cos() + (h/sin()cos()) - h/tan())
                        // position = a - d + k

                        servoHeading = -((robotHeading * 0.75) / Math.PI) + 0.52;

                        pitch.setPosition(1);
                        heading.setPosition(servoHeading);

                        int angle = (int)(Math.round((robotHeading + turn / 2.0) * 100));
                        if (angle < 0)
                            angle = 0;
                        if (angle > table.length - 1)
                            angle = table.length - 1;
                        int position = table[angle];


                        if (position > -2100)
                            setExtendPosition(position);
                        else
                            setExtendPosition(-2100);
                        flip.setTargetPosition(FLIP_INTAKE);
                    } else {
                        setExtendPosition(0);
                        heading.setPosition(0.52);
                        pitch.setPosition(0);
                    }

                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_SCORE;
                        drive.followTrajectorySequenceAsync(driveToScore);
                        setArmPosition(MAX_POSITION);
                    }
                    break;
                case DRIVE_TO_SCORE:
                    if (!drive.isBusy()) {
                        currentState = State.SCORING;
                        setArmPosition(SCORED_POSITION);
                    } if (arm.getCurrentPosition() < -600)
                        flip.setTargetPosition(FLIP_SCORE);
                    break;
                case SCORING:
                    if (atArmPosition(SCORED_POSITION)) {
                        setArmPosition(-600);
                        flip.setTargetPosition(FLIP_INTAKE);
                        samplesScored++;
                        if (samplesScored == 5) {
                            currentState = State.PARK;
                            drive.followTrajectorySequenceAsync(park);
                        } else {
                            currentState = State.RETURN;
                            drive.followTrajectorySequenceAsync(returnToZone);
                        }
                    }
                    break;
                case RETURN:
                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_SCORE;
                        drive.followTrajectorySequenceAsync(driveToScore);
                        setArmPosition(MAX_POSITION);
                    }
                    else if (atFlipPosition(0))
                        setArmPosition(GRAB_POSITION);
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    if (atFlipPosition(0))
                        setArmPosition(0);
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
