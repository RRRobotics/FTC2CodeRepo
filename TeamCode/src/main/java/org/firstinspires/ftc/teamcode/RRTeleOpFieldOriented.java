package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */

@TeleOp(group = "advanced")
@Config
public class RRTeleOpFieldOriented extends LinearOpMode {
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
    private ArmState armState = ArmState.X;
    public static double Kp = 0.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;

    enum IntakeMode {
        OFF, NEAR, LEFT, RIGHT
    }

    IntakeMode state = IntakeMode.OFF;
    enum ArmState {
        TRIANGLE, CIRCLE, X, SQUARE
    }

    CustomPID armPID;

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
        pitch1 = hardwareMap.get(Servo.class, "pitch1");
        pitch2 = hardwareMap.get(Servo.class, "pitch2");
        pitch2.setDirection(Servo.Direction.REVERSE);
        heading = hardwareMap.get(CRServo.class, "heading");
        grabber = hardwareMap.get(CRServo.class, "grabber");
        bumper = hardwareMap.get(Servo.class, "bumper");
        bumper.setDirection(Servo.Direction.REVERSE);
        telemetry.addData("initialization:", "is a success");
        telemetry.update();

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armPID = new CustomPID(arm, 0.0004, 0.0, 0.00002, 0);

        offset = 0;
        flipOffset = 0;
        int target = 0;

        // get pose or make new one
//        if (RRAuto3.endPose == null)
//            RRAuto3.endPose = new Pose2d();
//        drive.setPoseEstimate(RRAuto3.endPose);
        drive.setPoseEstimate(new Pose2d(45.00, -60.00, Math.toRadians(0)));

        waitForStart();
        pitch1.setPosition(0);
        pitch2.setPosition(0);

//        flip.setPower(1);
//        setFlipPosition(0);
//        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        flip.setDirection(DcMotorSimple.Direction.REVERSE);
        bumper.setPosition(0);


        if (isStopRequested()) return;

        double speed_factor = 0.4;
        boolean raising = false;
        boolean grab = false;
        boolean autoScore = false;
        boolean flipSides = false;
        boolean lowering = false;

        ElapsedTime timer = new ElapsedTime();

        ElapsedTime game = new ElapsedTime();


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("speed_factor:", speed_factor);
            telemetry.addData("arm position", arm.getCurrentPosition());
            telemetry.addData("extend position", extend.getCurrentPosition());
            telemetry.addData("flip position", flip.getCurrentPosition());
            telemetry.addData("flip target", flip.getTargetPosition());
            telemetry.addData("flip offset", flipOffset);
            telemetry.addData("state", state);
            telemetry.update();

//            armPID.setKp(Kp);
//            armPID.setKi(Ki);
//            armPID.setKd(Kd);

            dashboardTelemetry.addData("target", armPID.getTarget());
            dashboardTelemetry.addData("actual", -arm.getCurrentPosition());
            dashboardTelemetry.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            TrajectorySequence driveToScore = drive.trajectorySequenceBuilder(poseEstimate)
                    .lineToLinearHeading(new Pose2d(-2, -36.00, Math.toRadians(90.00)))
                    .build();

            // read robot heading from pose
            double headingRobot = drive.getPoseEstimate().getHeading();

            if (game.seconds() > 90 && game.seconds() < 91) {
                gamepad1.rumble(500);
            }

            // dpad changes intake mode
            if (gamepad1.dpad_up) {
                state = IntakeMode.OFF;
                extend.setTargetPosition(0);
                extend.setPower(1);
                extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pitch1.setPosition(0);
                pitch2.setPosition(0);
            } if (gamepad1.dpad_down) {
                state = IntakeMode.NEAR;
                // snap heading
//                if (headingRobot <= Math.toRadians(180)) {
//                    drive.turn(-headingRobot - Math.toRadians(5));
//                } else {
//                    drive.turn(Math.toRadians(365) - headingRobot);
//                }
            } if (gamepad1.dpad_left) {
                state = IntakeMode.LEFT;
                // snap heading
//                if (headingRobot >= Math.toRadians(270)) {
//                    drive.turn(-headingRobot + Math.toRadians(275));
//                } else if (headingRobot <= Math.toRadians(90)) {
//                    drive.turn(-headingRobot - Math.toRadians(95));
//                } else {
//                    drive.turn(Math.toRadians(275) - headingRobot);
//                }
            } if (gamepad1.dpad_right) {
                state = IntakeMode.RIGHT;
                // snap heading
//                if (headingRobot <= Math.toRadians(90)) {
//                    drive.turn(Math.toRadians(95) - headingRobot);
//                } else if (headingRobot >= Math.toRadians(270)) {
//                    drive.turn(Math.toRadians(455) - headingRobot);
//                } else {
//                    drive.turn(Math.toRadians(95) - headingRobot);
//                }
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

            // intake r 62
            // Spin intake, raise/lower head
            if (gamepad1.right_bumper) {
                pitch1.setPosition(1);
                pitch2.setPosition(1);
                grabber.setPower(1);
            } else if (gamepad1.left_bumper) {
                pitch1.setPosition(0.5);
                pitch2.setPosition(0.5);
                if (!lowering) {
                    lowering = true;
                    timer.reset();
                } if (timer.milliseconds() > 300) {
                    grabber.setPower(-1);
                }
            } else {
                grabber.setPower(0);
            }
            if (!(gamepad1.left_bumper || gamepad1.right_bumper)) {
                if (state == IntakeMode.OFF) {
                    pitch1.setPosition(0);
                    pitch2.setPosition(0);
                } else if (extend.getCurrentPosition() < -50) {
                    pitch1.setPosition(0.6);
                    pitch2.setPosition(0.6);
                }
                lowering = false;
            }

            // increase speed when right trigger
            if (gamepad1.right_trigger > 0.5)
                speed_factor = 1;
            else if (gamepad1.left_trigger > 0.5)
                speed_factor = 0.3;
            else
                speed_factor = 0.6;

            // auto score
//            if (gamepad1.left_trigger > 0.5 && !autoScore) {
//                autoScore = true;
//                drive.followTrajectorySequenceAsync(driveToScore);
//            } if (autoScore && !drive.isBusy()) {
//                autoScore = false;
//                Pose2d pose = drive.getPoseEstimate();
//                drive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), pose.getHeading() - Math.toRadians(90)));
//            }

            controlHeading(gamepad1.right_stick_x);

            Vector2d input;
            if (state == IntakeMode.OFF) {

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
//                    int position = arm.getCurrentPosition();
//                    int armtarget = arm.getTargetPosition();
//                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    arm.setTargetPosition(position - (int) (60 * speed_factor));
//                    offset -= armtarget - arm.getTargetPosition();
                    armPID.setTarget(armPID.getTarget() - 10);
                }
                // decrease arm offset
                if (gamepad1.share && gamepad1.cross) {
                    armPID.setTarget(armPID.getTarget() + 10);
                }
                // increase flip offset
                if (gamepad1.share && gamepad1.circle) {
                    int position = flip.getCurrentPosition();
                    target = flip.getTargetPosition();
                    flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setFlipPosition(position + 1);
                    flipOffset -= flip.getTargetPosition() - target - target;
                }
                // decrease flip offset
                if (gamepad1.share && gamepad1.square) {
                    int position = flip.getCurrentPosition();
                    target = flip.getTargetPosition();
                    flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setFlipPosition(position - 1);
                    flipOffset += flip.getTargetPosition() - target - target;
                }
                telemetry.addData("flip target", flip.getTargetPosition());
                telemetry.addData("target", target);

                //Arm Code
                controlArm(gamepad1.triangle, gamepad1.circle, gamepad1.cross, gamepad1.square);

                if (extend.getCurrentPosition() > -10)
                    extend.setPower(0.1);

            } else if (state == IntakeMode.NEAR) {


                // Only strafe drive code
                input = new Vector2d(0, -gamepad1.left_stick_x * speed_factor
                ).rotated(-headingRobot);
                drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), 0));

                // heading makes grabber turn by 3/4 of heading
                // grabber goes 5/2x faster than servo
                // make grabber go 3/10 of heading input
//                grabber.setPower(-gamepad1.right_stick_x * 3.0 / 10.0);

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

    public void setExtendPosition(int position) {
        setExtendPosition(position, 1);
    }
    public void setExtendPosition(int position, double power) {
        extend.setPower(power);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setTargetPosition(position);
    }

    public boolean atFlipPosition(int target) {
        return Math.abs(target - flip.getCurrentPosition()) < 20;
    }

    public boolean atArmPosition(int target) {
        return Math.abs(target - arm.getCurrentPosition()) < 50;
    }

    public void setFlipPosition(int position) {
        flip.setPower(1);
        flip.setTargetPosition(position + flipOffset);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void controlHeading(double joystick) {
        if (state != IntakeMode.OFF) {
            heading.setPower(joystick);
        }
    }

    public void controlArm(boolean triangle, boolean circle, boolean x, boolean square) {
        armPID.update();

        if (triangle && !gamepad1.share) {
            armState = ArmState.TRIANGLE;
        } else if (circle && !gamepad1.share) {
            armState = ArmState.CIRCLE;
        } else if (x && !gamepad1.share) {
            armState = ArmState.X;
        } else if (square && !gamepad1.share) {
            armState = ArmState.SQUARE;
        }

        if (armState == ArmState.X) {
            bumper.setPosition(0);
            armPID.setTarget(0);
        }

        if (armState == ArmState.SQUARE) {
            bumper.setPosition(0);
            armPID.setTarget(GRAB_POSITION);
        }

        if (armState == ArmState.TRIANGLE) {
            bumper.setPosition(1);
            armPID.setTarget(MAX_POSITION);
        }

        if (armState == ArmState.CIRCLE) {
            bumper.setPosition(1);
            armPID.setTarget(SCORED_POSITION);
        }


    }
}