package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
    private final int GRAB_POSITION = -175;
    private final int MAX_POSITION = -2092;
    private final int SCORED_POSITION = -1324;
    private final int MAX_EXTEND = -2160;
    private final int FLIP_SCORE = 0;
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
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flip.setTargetPosition(0);
        flip.setPower(1);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        offset = 0;

        // get pose or make new one
        if (RRAutoOdometry.endPose == null)
            RRAutoOdometry.endPose = new Pose2d();
        drive.setPoseEstimate(RRAutoOdometry.endPose);

        waitForStart();
        pitch.setPosition(1);
        arm.setTargetPosition(0);

        if (isStopRequested()) return;

        double speed_factor = 0.4;
        IntakeMode state = IntakeMode.OFF;

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("speed_factor:", speed_factor);
            telemetry.addData("arm position", arm.getCurrentPosition());
            telemetry.addData("extend", extend.getCurrentPosition());
            telemetry.addData("heading position", heading.getPosition());
            telemetry.addData("state", state);
            telemetry.addData("flip", flip.getCurrentPosition());
            telemetry.update();
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Change intake mode to dpad
            if (gamepad1.dpad_up) {
                state = IntakeMode.OFF;
                extend.setTargetPosition(0);
                extend.setPower(1);
                extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pitch.setPosition(1);
                heading.setPosition(0.52);
            } if (gamepad1.dpad_left)
                state = IntakeMode.LEFT;
            if (gamepad1.dpad_right)
                state = IntakeMode.RIGHT;
            if (gamepad1.dpad_down)
                state = IntakeMode.NEAR;

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
                pitch.setPosition(0);
            } else if (gamepad1.left_bumper) {
                intakeR.setPower(1);
                intakeL.setPower(-1);
                pitch.setPosition(1);
            }
            if (!(gamepad1.left_bumper || gamepad1.right_bumper)) {
                intakeR.setPower(0);
                intakeL.setPower(0);
                pitch.setPosition(1);
            }

            // increase speed when right trigger
            if (gamepad1.right_trigger > 0.5)
                speed_factor = 1;
            else
                speed_factor = 0.4;

            Vector2d input;
            if (state == IntakeMode.OFF) {
                heading.setPosition(0.52);

                // drive code
                input = new Vector2d(
                        -gamepad1.left_stick_y * speed_factor,
                        -gamepad1.left_stick_x * speed_factor
                ).rotated(-poseEstimate.getHeading());

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
                    arm.setTargetPosition(position - 10);
                    offset -= target - arm.getTargetPosition();
                }
                // decrease arm offset
                if (gamepad1.share && gamepad1.cross) {
                    int position = arm.getCurrentPosition();
                    int target = arm.getTargetPosition();
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setTargetPosition(position + 10);
                    offset -= target - arm.getTargetPosition();
                }

                //Arm Code
                if (gamepad1.cross && !gamepad1.share) {
                    setArmPosition(0);
                    //Floor position - Zero
                } if (gamepad1.triangle && !gamepad1.share) {
                    setArmPosition(MAX_POSITION);
                    flip.setTargetPosition(FLIP_SCORE);
                    //Raised - ready to score
                } if (gamepad1.square) {
                    //Pickup position
                    setArmPosition(GRAB_POSITION);
                    flip.setTargetPosition(FLIP_INTAKE);
                } if (gamepad1.circle) {
                    //Place - Pull down
                    setArmPosition(SCORED_POSITION);
                }

                if (extend.getCurrentPosition() > -10)
                    extend.setPower(0.1);


            } else if (state == IntakeMode.NEAR) {
                // Only strafe drive code
                input = new Vector2d(0, -gamepad1.left_stick_x * speed_factor
                ).rotated(-poseEstimate.getHeading());
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
                ).rotated(-poseEstimate.getHeading());
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
                ).rotated(-poseEstimate.getHeading());
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
}