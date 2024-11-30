package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RRTeleOp")
public class RRTeleOp extends OpMode {

    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor Arm;
    Servo Extend;
    Servo pitch;
    CRServo intakeR;
    CRServo intakeL;
    private double speed_factor = 0.4;
    private int offset;
    private final int GRAB_POSITION = -1200;
    private final int MAX_POSITION = -5700;
    private final int SCORED_POSITION = -4000;


    @Override
    public void init() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Extend = hardwareMap.get(Servo.class, "Extend");
        pitch = hardwareMap.get(Servo.class, "pitch");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        telemetry.addData("initialization:", "is a success");
        telemetry.update();
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        offset = 0;
    }

    @Override
    public void loop() {
        telemetry.addData("speed_factor:", speed_factor);
        telemetry.addData("arm position", Arm.getCurrentPosition());
        telemetry.addData("arm target", Arm.getTargetPosition());
        telemetry.addData("offset", offset);
        telemetry.update();
        if (gamepad1.right_trigger > 0.5)
            speed_factor = 1;
        else
            speed_factor = 0.4;

        // Drive
        FL.setPower((gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) - (gamepad1.left_stick_x * 0.6));
        FR.setPower(-(gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) - (gamepad1.left_stick_x * 0.6));
        BL.setPower((gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) + (gamepad1.left_stick_x * 0.6));
        BR.setPower(-(gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) + (gamepad1.left_stick_x * 0.6));

        if (gamepad1.share && gamepad1.triangle) {
            int position = Arm.getCurrentPosition();
            int target = Arm.getTargetPosition();
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setTargetPosition(position - 10);
            offset -= target - Arm.getTargetPosition();
        }
        if (gamepad1.share && gamepad1.cross) {
            int position = Arm.getCurrentPosition();
            int target = Arm.getTargetPosition();
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setTargetPosition(position + 10);
            offset -= target - Arm.getTargetPosition();

        }
        if (gamepad1.touchpad) {
            int temp = offset;
            Arm.setTargetPosition(Arm.getTargetPosition() - temp);
            offset = 0;
        }

        //Arm Code
        if (gamepad1.cross && !gamepad1.share) {
            setArmPosition(0);
            //Floor position - Zero
        }
        if (gamepad1.triangle && !gamepad1.share) {
            setArmPosition(MAX_POSITION);
            //Raised - ready to score
        }
        if (gamepad1.square) {
            //Pickup position
            setArmPosition(GRAB_POSITION);
        }
        if (gamepad1.circle) {
            //Place - Pull down
            setArmPosition(SCORED_POSITION);
        }

        //Floor Pickup
        if (gamepad1.dpad_right) {
            Extend.setPosition(0.3);
        }
        if (gamepad1.dpad_left) {
            Extend.setPosition(0);
        }
        if (gamepad1.dpad_up) {
            pitch.setPosition(0);
        }
        if (gamepad1.dpad_down) {
            pitch.setPosition(1);
        }

        // Spinny intake
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

    public void setArmPosition(int position) {
        setArmPosition(position, 1);
    }
    public void setArmPosition(int position, double power) {
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(power);
        Arm.setTargetPosition(position + offset);
    }
}