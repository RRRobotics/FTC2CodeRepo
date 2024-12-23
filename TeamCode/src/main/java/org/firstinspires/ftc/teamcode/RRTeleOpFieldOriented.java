package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */

@TeleOp(group = "advanced")
public class RRTeleOpFieldOriented extends LinearOpMode {
    DcMotor arm, extend, flip;
    Servo pitch, heading;
    CRServo intakeR, intakeL;

    private double speed_factor = 0.4;
    private int offset;
    private final int GRAB_POSITION = 0;
    private final int MAX_POSITION = -2010;
    private final int SCORED_POSITION = -1350;
    private final int MAX_EXTEND = -2160;
    private final int FLIP_SCORE = 1350;
    private final int FLIP_INTAKE = 0;
    enum IntakeMode {
        OFF, NEAR, LEFT, RIGHT
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, false);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm = hardwareMap.get(DcMotor.class, "arm");
        extend = hardwareMap.get(DcMotor.class, "extend");
        flip = hardwareMap.get(DcMotor.class, "flip");
        pitch = hardwareMap.get(Servo.class, "pitch");
        heading = hardwareMap.get(Servo.class, "heading");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        telemetry.addData("initialization:", "is a success");
        telemetry.update();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        offset = 0;

        // get pose or make new one
//        if (RRAuto3.endPose == null)
//            RRAuto3.endPose = new Pose2d();
//        drive.setPoseEstimate(RRAuto3.endPose);
        drive.setPoseEstimate(new Pose2d(45.00, -60.00, Math.toRadians(90)));

        waitForStart();
        pitch.setPosition(0);
//        arm.setTargetPosition(0);
        flip.setPower(1);
        flip.setTargetPosition(0);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (isStopRequested()) return;

        double speed_factor = 0.4;
        IntakeMode state = IntakeMode.OFF;
        boolean raising = false;
        boolean grab = false;
        boolean autoScore = false;

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("speed_factor:", speed_factor);
            telemetry.addData("arm position", arm.getCurrentPosition());
            telemetry.addData("extend position", extend.getCurrentPosition());
            telemetry.addData("flip position", flip.getCurrentPosition());
            telemetry.addData("heading position", heading.getPosition());
            telemetry.addData("state", state);
            telemetry.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            TrajectorySequence driveToScore = drive.trajectorySequenceBuilder(poseEstimate)
                    .lineToLinearHeading(new Pose2d(-2, -36.00, Math.toRadians(90.00)))
                    .build();

            // read robot heading from pose
            double headingRobot = drive.getPoseEstimate().getHeading();

            // dpad changes intake mode
            if (gamepad1.dpad_up) {
                state = IntakeMode.OFF;
                extend.setTargetPosition(0);
                extend.setPower(1);
                extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pitch.setPosition(0);
                heading.setPosition(0.52);
            } if (gamepad1.dpad_down) {
                state = IntakeMode.NEAR;
                // snap heading
                if (headingRobot <= Math.toRadians(180)) {
                    drive.turn(-headingRobot - Math.toRadians(5));
                } else {
                    drive.turn(Math.toRadians(365) - headingRobot);
                }
            } if (gamepad1.dpad_left) {
                state = IntakeMode.LEFT;
                // snap heading
                if (headingRobot >= Math.toRadians(270)) {
                    drive.turn(-headingRobot + Math.toRadians(275));
                } else if (headingRobot <= Math.toRadians(90)) {
                    drive.turn(-headingRobot - Math.toRadians(95));
                } else {
                    drive.turn(Math.toRadians(275) - headingRobot);
                }
            } if (gamepad1.dpad_right) {
                state = IntakeMode.RIGHT;
                // snap heading
                if (headingRobot <= Math.toRadians(90)) {
                    drive.turn(Math.toRadians(95) - headingRobot);
                } else if (headingRobot >= Math.toRadians(270)) {
                    drive.turn(Math.toRadians(455) - headingRobot);
                } else {
                    drive.turn(Math.toRadians(95) - headingRobot);
                }
            }

            // reset arm offset
            if (gamepad1.touchpad) {
                int temp = offset;
                arm.setTargetPosition(arm.getTargetPosition() - temp);
                offset = 0;
            }

            // reset robot heading
            if (gamepad1.options){
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0));
            }

            // Spin intake, raise/lower head
            if (gamepad1.right_bumper) {
                intakeR.setPower(-1);
                intakeL.setPower(1);
                pitch.setPosition(1);
            } else if (gamepad1.left_bumper) {
                intakeR.setPower(1);
                intakeL.setPower(-1);
                pitch.setPosition(0);
            }
            if (!(gamepad1.left_bumper || gamepad1.right_bumper)) {
                intakeR.setPower(0);
                intakeL.setPower(0);
                pitch.setPosition(0);
            }

            // increase speed when right trigger
            if (gamepad1.right_trigger > 0.5)
                speed_factor = 1;
            else
                speed_factor = 0.4;

            // auto score
//            if (gamepad1.left_trigger > 0.5 && !autoScore) {
//                autoScore = true;
//                drive.followTrajectorySequenceAsync(driveToScore);
//            } if (autoScore && !drive.isBusy()) {
//                autoScore = false;
//                Pose2d pose = drive.getPoseEstimate();
//                drive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), pose.getHeading() - Math.toRadians(90)));
//            }

            Vector2d input;
            if (state == IntakeMode.OFF) {
                heading.setPosition(0.52);

                // drive code
                input = new Vector2d(
                        -gamepad1.left_stick_y * speed_factor,
                        -gamepad1.left_stick_x * speed_factor
                ).rotated(-headingRobot);

                drive.setWeightedDrivePower(new Pose2d(
                                input.getX(), input.getY(),
                                -gamepad1.right_stick_x * speed_factor
                        )
                );

                // increase arm offset
                if (gamepad1.share && gamepad1.triangle) {
                    int position = arm.getCurrentPosition();
                    int target = arm.getTargetPosition();
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setTargetPosition(position - (int)(40 * speed_factor));
                    offset -= target - arm.getTargetPosition();
                }
                // decrease arm offset
                if (gamepad1.share && gamepad1.cross) {
                    int position = arm.getCurrentPosition();
                    int target = arm.getTargetPosition();
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setTargetPosition(position + (int)(40 * speed_factor));
                    offset -= target - arm.getTargetPosition();
                }

                //Arm Code
                if (gamepad1.cross && !gamepad1.share) {
                    setArmPosition(-600);
                    raising = false;
                    grab = false;
                    //Floor position - Zero
                } if (gamepad1.triangle && !gamepad1.share) {
                    setArmPosition(MAX_POSITION);
                    raising = true;
                    grab = false;
                    //Raised - ready to score
                } if (gamepad1.square) {
                    //Pickup position
                    setArmPosition(-600);
                    raising = false;
                    grab = true;
                } if (gamepad1.circle) {
                    //Place - Pull down
                    setArmPosition(SCORED_POSITION);
                    raising = true;
                    grab = false;
                }
                if (raising && arm.getCurrentPosition() < -600)
                    flip.setTargetPosition(FLIP_SCORE);
                if (!raising) {
                    flip.setTargetPosition(FLIP_INTAKE);
                    if (atFlipPosition(FLIP_INTAKE)) {
                        if (grab)
                            setArmPosition(GRAB_POSITION);
                        else
                            setArmPosition(0);
                    }
                }

                if (extend.getCurrentPosition() > -10)
                    extend.setPower(0.1);


            } else if (state == IntakeMode.NEAR) {


                // Only strafe drive code
                input = new Vector2d(0, -gamepad1.left_stick_x * speed_factor
                ).rotated(-headingRobot);
                drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), 0));

                // pivot intake head
                heading.setPosition(heading.getPosition() - (gamepad1.right_stick_x * 0.01));

                // move extend arm with joystick
                extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (extend.getCurrentPosition() > MAX_EXTEND) {
                    extend.setPower(gamepad1.left_stick_y);
                } else {
                    extend.setPower(0.1);
                    extend.setTargetPosition(MAX_EXTEND);
                }

            } else if (state == IntakeMode.RIGHT) {

                input = new Vector2d(-gamepad1.left_stick_y * speed_factor,0
                ).rotated(-headingRobot);
                drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), 0));

                // pivot intake head
                heading.setPosition(heading.getPosition() - (gamepad1.right_stick_x * 0.01));

                // move extend arm with joystick
                extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (extend.getCurrentPosition() > MAX_EXTEND) {
                    extend.setPower(gamepad1.left_stick_x);
                } else {
                    extend.setPower(0.1);
                    extend.setTargetPosition(MAX_EXTEND);
                }
            } else if (state == IntakeMode.LEFT) {

                input = new Vector2d(-gamepad1.left_stick_y * speed_factor,0
                ).rotated(-headingRobot);
                drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), 0));

                // pivot intake head
                heading.setPosition(heading.getPosition() + (gamepad1.right_stick_x * 0.01));

                // move extend arm with joystick
                extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (extend.getCurrentPosition() > MAX_EXTEND) {
                    extend.setPower(-gamepad1.left_stick_x);
                } else {
                    extend.setPower(0.1);
                    extend.setTargetPosition(MAX_EXTEND);
                }
            }
            drive.update();
        }
    }
    public void setArmPosition(int position) {
        setArmPosition(position, 1);
    }
    public void setArmPosition(int position, double power) {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
        arm.setTargetPosition(position + offset);
    }

    public void setExtendPosition(int position) {
        setExtendPosition(position, 1);
    }
    public void setExtendPosition(int position, double power) {
        extend.setPower(position);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setTargetPosition(position);
    }

    public boolean atFlipPosition(int target) {
        return Math.abs(target - flip.getCurrentPosition()) < 10;
    }
}