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
    DcMotor arm, extend;
    Servo pitch, heading;
    CRServo intakeR, intakeL;

    private double speed_factor = 0.4;
    private int offset;
    private final int GRAB_POSITION = -219;
    private final int MAX_POSITION = -2092;
    private final int SCORED_POSITION = -1324;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, false);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm = hardwareMap.get(DcMotor.class, "arm");
        extend = hardwareMap.get(DcMotor.class, "extend");
        pitch = hardwareMap.get(Servo.class, "pitch");
        heading = hardwareMap.get(Servo.class, "heading");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        telemetry.addData("initialization:", "is a success");
        telemetry.update();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        offset = 0;

        // get pose or make new one
        if (RRAutoOdometry.endPose == null)
            RRAutoOdometry.endPose = new Pose2d();
        drive.setPoseEstimate(RRAutoOdometry.endPose);

        waitForStart();

        if (isStopRequested()) return;

        double speed_factor = 0.4;

        boolean intake_mode = false;
        boolean toggle = false;

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("speed_factor:", speed_factor);
            telemetry.addData("arm position", arm.getCurrentPosition());
            telemetry.addData("extend", extend.getCurrentPosition());
            telemetry.addData("heading position", heading.getPosition());
            telemetry.update();
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // increase speed when right trigger
            if (gamepad1.right_trigger > 0.5)
                speed_factor = 1;
            else
                speed_factor = 0.4;

            // toggle intake_mode with left trigger
            if (!toggle) {
                if (gamepad1.left_trigger > 0.5) {
                    intake_mode = !intake_mode;
                    toggle = true;
                }
            }
            if (gamepad1.left_trigger < 0.5)
                toggle = false;


        // drive code
            Vector2d input;
            if (intake_mode) {
                // Only strafe
                input = new Vector2d(0, -gamepad1.left_stick_x * speed_factor
                ).rotated(-poseEstimate.getHeading());

                drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), 0));
            } else {
                // Normal drive
                input = new Vector2d(
                        -gamepad1.left_stick_y * speed_factor,
                        -gamepad1.left_stick_x * speed_factor
                ).rotated(-poseEstimate.getHeading());

                drive.setWeightedDrivePower(new Pose2d(
                                input.getX(), input.getY(),
                                -gamepad1.right_stick_x * speed_factor
                        )
                );
            }

            // Update everything
            drive.update();

            // increase arm offset
            if (gamepad1.share && gamepad1.triangle && !intake_mode) {
                int position = arm.getCurrentPosition();
                int target = arm.getTargetPosition();
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition(position - 10);
                offset -= target - arm.getTargetPosition();
            }
            // decrease arm offset
            if (gamepad1.share && gamepad1.cross && !intake_mode) {
                int position = arm.getCurrentPosition();
                int target = arm.getTargetPosition();
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition(position + 10);
                offset -= target - arm.getTargetPosition();
            }
            // reset arm offset
            if (gamepad1.touchpad) {
                int temp = offset;
                arm.setTargetPosition(arm.getTargetPosition() - temp);
                offset = 0;
            }
            // reset heading
            if (gamepad1.options){
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0));
            }

            //Arm Code
            if (gamepad1.cross && !gamepad1.share && !intake_mode) {
                setArmPosition(0);
                //Floor position - Zero
            }
            if (gamepad1.triangle && !gamepad1.share && !intake_mode) {
                setArmPosition(MAX_POSITION);
                //Raised - ready to score
            }
            // pivot intake head
            if (intake_mode) {
                // turn left
                if (gamepad1.square) {
                    heading.setPosition(heading.getPosition() + 0.005);
                }
                // turn right
                if (gamepad1.circle) {
                    heading.setPosition(heading.getPosition() - 0.005);
                }
                // center
                if (gamepad1.triangle || gamepad1.cross) {
                    heading.setPosition(0.52);
                }
            } else {
                heading.setPosition(0.52);
                if (gamepad1.square) {
                    //Pickup position
                    setArmPosition(GRAB_POSITION);
                }
                if (gamepad1.circle) {
                    //Place - Pull down
                    setArmPosition(SCORED_POSITION);
                }
            }
            if (intake_mode) {
                // move extend arm with joystick
                extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extend.setPower(gamepad1.left_stick_y * speed_factor);
            } else {
                // move extend arm (max, min) with dpad
                if (gamepad1.dpad_right) {
                    extend.setTargetPosition(-2000);
                    extend.setPower(1);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (gamepad1.dpad_left) {
                    extend.setTargetPosition(0);
                    extend.setPower(1);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }

            // raise/lower intake head
            if (gamepad1.dpad_up) {
                pitch.setPosition(0);
            }
            if (gamepad1.dpad_down) {
                pitch.setPosition(1);
            }

            // Spin intake
            if (gamepad1.right_bumper) {
                intakeR.setPower(-1);
                intakeL.setPower(1);
            }
            if (gamepad1.left_bumper) {
                intakeR.setPower(1);
                intakeL.setPower(-1);
            }
            if (!(gamepad1.left_bumper || gamepad1.right_bumper)) {
                intakeR.setPower(0);
                intakeL.setPower(0);
            }
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