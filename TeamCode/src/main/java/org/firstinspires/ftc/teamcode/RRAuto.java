package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RRAuto")
public class RRAuto extends LinearOpMode {

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


        FL.setTargetPosition(FL.getCurrentPosition());
        BL.setTargetPosition(BL.getCurrentPosition());
        FR.setTargetPosition(FR.getCurrentPosition());
        BR.setTargetPosition(BR.getCurrentPosition());

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();


        // lift arm
        setArmPosition(-590, 0.5);
        sleep(1500);

        // drive forward
        forward(3.5);
        sleep(1000);

        //Score Piece
        sleep(500);
        setArmPosition(-320, 1);
        sleep(500);

        // drive forward
        forward(.3);
        sleep(1000);

        // drive backward
        forward(-1.4);
        sleep(1000);

        // reset arm
        setArmPosition(0);
        sleep(1500);

    }


    public void setArmPosition(int position) {
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(0.5);
        Arm.setTargetPosition(position);
    }
    public void setArmPosition(int position, double power) {
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(power);
        Arm.setTargetPosition(position);
    }
    public void turnRight(double rotations) {
        move(rotations, -1, -1, -1, -1);
    }
    public void forward(double rotations) {
        move(rotations, -1, -1, 1, 1);
    }

    public void strafe(double rotations) {
        move(rotations, -1, 1, -1, 1);
    }
    public void rotate(double rotations) {
        move(rotations, 1, 1, 1, 1);
    }
    private void move(double rotations, int fl, int bl, int fr, int br) {
        FL.setPower(0.5);
        BL.setPower(0.5);
        FR.setPower(0.5);
        BR.setPower(0.5);
        FL.setTargetPosition(FL.getCurrentPosition() + (int)(rotations * 360 * fl));
        BL.setTargetPosition(BL.getCurrentPosition() + (int)(rotations * 360 * bl));
        FR.setTargetPosition(FR.getCurrentPosition() + (int)(rotations * 360 * fr));
        BR.setTargetPosition(BR.getCurrentPosition() + (int)(rotations * 360 * br));

    }

}
