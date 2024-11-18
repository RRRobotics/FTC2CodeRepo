package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.*;

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
    private boolean sub_pickup = false;
    private boolean specimen_score = false;


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

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(30, 30, Math.toRadians(90)))
                .build();

        Trajectory myTraj2 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(60 * Math.sqrt(2), 0, Math.toRadians(180)))
                .build();

        Trajectory myTraj3 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(30, -30, Math.toRadians(270)))
                .build();


        Trajectory myTraj4 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();
        drive.setPoseEstimate(new Pose2d());

        waitForStart();

        drive.followTrajectory(myTrajectory);
        drive.followTrajectory(myTraj2);
        drive.followTrajectory(myTraj3);
        drive.followTrajectory(myTraj4);

        if(isStopRequested()) return;

    }

}
