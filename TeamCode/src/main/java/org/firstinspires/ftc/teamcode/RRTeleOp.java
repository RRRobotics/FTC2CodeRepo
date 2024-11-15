package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

@TeleOp(name = "RRTeleOp")
public class RRTeleOp extends OpMode {

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



    @Override
    public void init() {
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

    }

    @Override
    public void loop() {
        telemetry.addData("speed_factor:", speed_factor);
        telemetry.addData("arm position", Arm.getCurrentPosition());
        telemetry.addData("Extend position", Extend.getPosition());
        telemetry.addData("Pivot position", Pivot.getPosition());
        if (gamepad1.right_trigger > 0.5)
            speed_factor = 1;
        else
            speed_factor = 0.4;

        //Drive Code
        FL.setPower((gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) - (gamepad1.left_stick_x * 0.6));
        FR.setPower(-(gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) - (gamepad1.left_stick_x * 0.6));
        BL.setPower((gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) + (gamepad1.left_stick_x * 0.6));
        BR.setPower(-(gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) + (gamepad1.left_stick_x * 0.6));

        //Arm Code
        if (gamepad1.cross) {
            setArmPosition(0);
            //Floor position - Zero
        }
        if (gamepad1.triangle) {
            setArmPosition(-580);
            //Raised - ready to score
        }
        if (gamepad1.square) {
            //Pickup position
            setArmPosition(-190);
        }
        if (gamepad1.circle) {
            //Place - Pull down
            setArmPosition(-390);
        }

        if (gamepad1.share) {
            setArmPosition(-440, 0.1);
        }

        //Floor Pickup Code
        if (gamepad1.dpad_right) {
            Extend.setPosition(1);
        }
        if (gamepad1.dpad_left) {
            Extend.setPosition(0);
        }



        if (gamepad1.right_bumper) {
            Intake.setPower(-1);
        }
        if (gamepad1.left_bumper) {
            Intake.setPower(1);
        }
        if (!(gamepad1.left_bumper || gamepad1.right_bumper)) {
            Intake.setPower(0);
        }






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
}