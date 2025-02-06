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
import com.qualcomm.robotcore.hardware.PIDCoefficients;
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
public class NewEncoderTest extends LinearOpMode {
    DcMotor arm, extend, flip;
    Servo pitch, heading, push, bumper;

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
    public static int reference = 1000;


    enum IntakeMode {
        OFF, NEAR, LEFT, RIGHT
    }

    enum ArmState {
        TRIANGLE, CIRCLE, X, SQUARE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.get(DcMotor.class, "arm");
        flip = hardwareMap.get(DcMotor.class, "flip");
        extend = hardwareMap.get(DcMotor.class, "extend");


        waitForStart();

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setPower(0.5);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        flip.setPower(0.5);
//        flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        flip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        flip.setDirection(DcMotorSimple.Direction.REVERSE);

//        if (!isStopRequested()) return;

        double integralSum = 0.0;
        double lastError = 0;
        int oldReference = reference;
        boolean running = false;

        ElapsedTime timer = new ElapsedTime();

        ElapsedTime timer2 = new ElapsedTime();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("flip", flip.getCurrentPosition());
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.addData("arm target", arm.getTargetPosition());
            telemetry.addData("extend", extend.getCurrentPosition());

            int encoderPosition = -arm.getCurrentPosition();
            double error = reference - encoderPosition;

            dashboardTelemetry.addData("target", reference);
            dashboardTelemetry.addData("actual", encoderPosition);
            dashboardTelemetry.addData("error", error);
            dashboardTelemetry.addData("Kp", Kp);
            dashboardTelemetry.addData("Ki", Ki);
            dashboardTelemetry.addData("Kd", Kd);
            dashboardTelemetry.update();

            if (oldReference != reference) {
                running = true;
            } else if (Math.abs(error) < 10) {
                running = false;
            }
            if (!running && Math.abs(error) > 50) {
                running = true;
            }


//            Kp = 0.01;
//            Ki = 0.0;
//            Kd = 0.0001;

            double derivative = (double) (error - lastError) / (double) timer.seconds();

            if (running) {


                integralSum = integralSum + (error * timer.seconds());

                double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                arm.setPower(out);

                lastError = error;

                timer.reset();

            } else {
                arm.setPower(0);
            }

            oldReference = reference;

            System.out.println(running);

            telemetry.update();
        }
    }

}