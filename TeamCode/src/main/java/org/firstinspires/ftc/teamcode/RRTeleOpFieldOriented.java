package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.hardware.TouchSensor;
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
    DcMotorEx arm, extend, flip;
    Servo pitch1, pitch2, bumper;
    CRServo heading, grabber;
    TouchSensor bottom, top, side;

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
//    PIDFController armPID = new PIDFController(new PIDCoefficients(Kp, Ki, Kd));

    enum IntakeMode {
        OFF, NEAR, LEFT, RIGHT
    }

    IntakeMode state = IntakeMode.OFF;
    enum ArmState {
        TRIANGLE, CIRCLE, X, SQUARE
    }

    CustomPID armPID;
    ElapsedTime game;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, false);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        side = hardwareMap.touchSensor.get("side");
        telemetry.addData("initialization:", "is a success");
        telemetry.update();

        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armPID = new CustomPID(arm, 0.0004, 0.0, 0.00002, 0);

        offset = 0;
        flipOffset = 0;
        int target = 0;

        drive.setPoseEstimate(new Pose2d(45.00, -60.00, Math.toRadians(0)));

        waitForStart();
        pitch1.setPosition(0.3);
        pitch2.setPosition(0.3);

        bumper.setPosition(0);

        ElapsedTime eject = new ElapsedTime();

        if (isStopRequested()) return;

        double speed_factor = 0.4;
        boolean lowering = false;

        ElapsedTime timer = new ElapsedTime();

        game = new ElapsedTime();

        boolean extended = false;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

//        long oldTime = System.nanoTime();
        while (opModeIsActive() && !isStopRequested()) {
//            long newTime = System.nanoTime();
//            System.out.println((newTime - oldTime) / 1000);
//            oldTime = newTime;
//            telemetry.addData("flip", flip.getCurrentPosition());
//            telemetry.addData("arm current", arm.getCurrent(CurrentUnit. AMPS));
//            telemetry.addData("extend current", extend.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("flip current", flip.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("arm position", armPID.getPosition());
//            telemetry.addData("arm target", armPID.getTarget());
//            telemetry.addData("homed", homed);
//            telemetry.addData("extend position", extend.getCurrentPosition());
//            telemetry.addData("extend target", extend.getTargetPosition());
//            telemetry.addData("extended", extended);
//            telemetry.addData("side", side.isPressed());
//            telemetry.addData("flip position", flip.getCurrentPosition());
//            telemetry.addData("flip target", flip.getTargetPosition());
//            telemetry.addData("flipped", flipped);
//            telemetry.addData("state", state);
//            telemetry.update();

//            armPID.setKp(Kp);
//            armPID.setKi(Ki);
//            armPID.setKd(Kd);

//            dashboardTelemetry.addData("target", armPID.getTarget());
//            dashboardTelemetry.addData("actual", armPID.getPosition());
//            dashboardTelemetry.update();

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
                pitch1.setPosition(0.3);
                pitch2.setPosition(0.3);
                grabber.setPower(0.6);
            } if (gamepad1.dpad_down) {
                state = IntakeMode.NEAR;
            } if (gamepad1.dpad_left) {
                state = IntakeMode.LEFT;
            } if (gamepad1.dpad_right) {
                state = IntakeMode.RIGHT;
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

            if (gamepad1.right_bumper) {
                pitch1.setPosition(1);
                pitch2.setPosition(1);
                grabber.setPower(1);
            } else if (gamepad1.left_bumper) {
                setExtendPosition(MAX_EXTEND, 1);
                if (!lowering) {
                    eject.reset();
                    lowering = true;
                } if (eject.milliseconds() > 100) {
                    pitch1.setPosition(0.8);
                    pitch2.setPosition(0.8);
                }
                if (eject.milliseconds() > 400) {
                    grabber.setPower(-1);
                }
            } else if (state == IntakeMode.OFF){
                setExtendPosition(0);
                grabber.setPower(0);
            } else {
                grabber.setPower(0);
                lowering = false;
            }
            if (!(gamepad1.left_bumper || gamepad1.right_bumper)) {
                if (state == IntakeMode.OFF) {
                    pitch1.setPosition(0.3);
                    pitch2.setPosition(0.3);
                } else if (extend.getCurrentPosition() < -100) {
                    pitch1.setPosition(0.6);
                    pitch2.setPosition(0.6);
                }
            }

            // increase speed when right trigger
            if (gamepad1.right_trigger > 0.5)
                speed_factor = 1;
            else if (gamepad1.left_trigger > 0.5)
                speed_factor = 0.3;
            else
                speed_factor = 0.6;


            controlHeading(gamepad1.right_stick_x);

            // extend rezero
            if (extend.getTargetPosition() >= 0 && !extended && state == IntakeMode.OFF && game.seconds() > 1) {
                if (side.isPressed()) {
                    extended = true;
                    extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    setFlipPosition(-5);
                } else {
                    extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    extend.setPower(0.7);
                }
            } else if (extend.getTargetPosition() >= 0 && extend.getCurrent(CurrentUnit.AMPS) > 2) {
                extended = false;
            }

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
//                    int position = armPID.getPosition();
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

                //Arm Code
                controlArm(gamepad1.triangle, gamepad1.circle, gamepad1.cross, gamepad1.square);

                if (extend.getCurrentPosition() == 0)
                    extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        extend.setTargetPosition(position);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean atFlipPosition(int target) {
        return Math.abs(target - flip.getCurrentPosition()) < 10;
    }
    public boolean atExtendPositionPrecise(int target) {
        return extend.getCurrentPosition() == target;
    }

    public boolean atArmPosition(int target) {
        return Math.abs(target - armPID.getPosition()) < 50;
    }

    public boolean atArmPositionRough(int target) {
        return Math.abs(target - armPID.getPosition()) < 300;
    }

    public void setFlipPosition(int position, double power) {
        flip.setPower(power);
        flip.setTargetPosition(position + flipOffset);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void setFlipPosition(int position) {
        setFlipPosition(position, 0.3);
    }


    public void controlHeading(double joystick) {
        if (state != IntakeMode.OFF) {
            heading.setPower(joystick);
        }
    }
    boolean homed = false;
    boolean flipped = false;

    ElapsedTime flipTimer = new ElapsedTime();
    public void controlArm(boolean triangle, boolean circle, boolean x, boolean square) {
        armPID.update();

//        if (bottom.isPressed() && !homed) {
//            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            armPID.setTarget(0);
//            homed = true;
//            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        } else if (!homed) {
//            setFlipPosition(FLIP_INTAKE);
//            if (atFlipPosition(FLIP_INTAKE)) {
//                armPID.setPower(0.3);
//            } else {
//                armPID.setTarget(-8000);
//            }
//        }
//
//        if ((atArmPositionRough(0) && !bottom.isPressed()) || (!atArmPositionRough(0) && !bottom.isPressed()) && armState == ArmState.X) {
//            homed = false;
//        }

        if (armPID.getTarget() >= 0 && !homed && game.seconds() > 1) {
            if (bottom.isPressed()) {
                homed = true;
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armPID.setTarget(0);
            } else {
                armPID.setPower(0.5);
            }
        }


        if (flip.getTargetPosition() >= 0 && !flipped) {
            if (top.isPressed()) {
                flipped = true;
                flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setFlipPosition(-5);
            } else {
                flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                flip.setPower(0.4);
            }
        } else if (flip.getTargetPosition() >= 0 && flip.getCurrent(CurrentUnit.AMPS) > 2) {
            flipped = false;
        }

//        if (flip.getTargetPosition() == FLIP_INTAKE && flip.getCurrentPosition() > -5 && !top.isPressed()) {
//            flipped = false;
//            flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            flip.setPower(1);
//        } else if (flip.getTargetPosition() == FLIP_INTAKE && top.isPressed() && !flipped) {
//            flipTimer.reset();
//            flipped = true;
//        } else if (flip.getTargetPosition() == FLIP_INTAKE && top.isPressed() && flipTimer.seconds() > 1) {
//            flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            setFlipPosition(0);
//        }

//        if (flip.getCurrentPosition() > -10 && flip.getTargetPosition() == FLIP_INTAKE) {
//            flip.setPower(0.1);
//        }



        if (triangle && !gamepad1.share) {
            armState = ArmState.TRIANGLE;
            setFlipPosition(FLIP_SCORE);
        } else if (circle && !gamepad1.share) {
            armState = ArmState.CIRCLE;
            setFlipPosition(FLIP_SCORE);
        } else if (x && !gamepad1.share) {
            armState = ArmState.X;
            setFlipPosition(FLIP_INTAKE);
            flipped = false;
            homed = false;
        } else if (square && !gamepad1.share) {
            armState = ArmState.SQUARE;
            setFlipPosition(FLIP_INTAKE);
            flipped = false;
        }


        if (armState == ArmState.X) {
            bumper.setPosition(0);
            armPID.setTarget(0);
//            if (flip.getCurrentPosition() > -10) {
//                setFlipPosition(FLIP_INTAKE, 1);
//            }
        }


        if (armState == ArmState.SQUARE) {
            bumper.setPosition(0);
            armPID.setTarget(GRAB_POSITION);
//            if (flip.getCurrentPosition() > -10) {
//                setFlipPosition(FLIP_INTAKE, 1);
//            }
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