package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
public class NewEncoderTest extends LinearOpMode {
    DcMotorEx arm, extend, flip;
    Servo pitch, heading, push, bumper;
    TouchSensor bottom;

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


    public static double kp = 0.0;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static int reference = 1000;


    enum IntakeMode {
        OFF, NEAR, LEFT, RIGHT
    }

    enum ArmState {
        TRIANGLE, CIRCLE, X, SQUARE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        flip = hardwareMap.get(DcMotorEx.class, "flip");
//        extend = hardwareMap.get(DcMotor.class, "extend");
        bottom = hardwareMap.touchSensor.get("bottom");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        CustomPID armPID = new CustomPID(arm, kp, ki, kd, reference);

        waitForStart();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        ElapsedTime timer = new ElapsedTime();

        flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flip.setPower(0.5);

        while (opModeIsActive() && !isStopRequested()) {

//            System.out.println(arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

            dashboardTelemetry.addData("target", reference);
            dashboardTelemetry.addData("actual", armPID.getPosition());
            dashboardTelemetry.addData("kp", kp);
            dashboardTelemetry.addData("ki", ki);
            dashboardTelemetry.addData("kd", kd);
            dashboardTelemetry.addData("running", armPID.isRunning());
            dashboardTelemetry.addData("flip", flip.getCurrentPosition());
            dashboardTelemetry.update();

//            reference = -(int) (6000 + Math.sin(timer.seconds() / 2) * 5000);

            armPID.setKp(kp);
            armPID.setKi(ki);
            armPID.setKd(kd);
            armPID.setTarget(reference);
            armPID.update();

            telemetry.update();
        }
    }

}