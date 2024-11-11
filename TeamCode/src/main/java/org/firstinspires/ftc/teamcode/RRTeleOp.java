package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RRTeleOp")
public class RRTeleOp extends OpMode {

    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor Arm;
    Servo Claw;
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
        Claw = hardwareMap.get(Servo.class, "Claw");
        telemetry.addData("initialization:", "is a success");
        telemetry.update();
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
    }

    @Override
    public void loop() {
        telemetry.addData("speed_factor:", speed_factor);
        telemetry.addData("arm position", Arm.getCurrentPosition());
        if (gamepad1.right_trigger > 0.5)
            speed_factor = 1;
        else
            speed_factor = 0.4;


        FL.setPower((gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) - (gamepad1.left_stick_x * 0.6));
        FR.setPower(-(gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) - (gamepad1.left_stick_x * 0.6));
        BL.setPower((gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) + (gamepad1.left_stick_x * 0.6));
        BR.setPower(-(gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) + (gamepad1.left_stick_x * 0.6));

        if (gamepad1.right_bumper) {
            //Close Claw
            Claw.setPosition(0.5);
            //gamepad1.rumble(1, 1, 1000);
        }
        if (gamepad1.left_bumper) {
            //Open Claw
            Claw.setPosition(0.1);
        }

        if (gamepad1.cross) {
            setArmPosition(0);
            sub_pickup = false;
            specimen_score = false;
            //Front Pick up position
        }
        if (gamepad1.triangle) {
            setArmPosition(-580);
            sub_pickup = false;
            specimen_score = false;
            //Mid Basket position
        }
        if (gamepad1.square) {
            //Back Pickup position
            setArmPosition(-190);
            sub_pickup = true;
            specimen_score = false;
        }
        if (gamepad1.circle) {
            setArmPosition(-390);
            specimen_score = true;
            sub_pickup = false;
        }

        if (gamepad1.dpad_up && !(gamepad1.left_trigger > 0.5)) {
            if (sub_pickup)
                setArmPosition(-1800);
            else if (specimen_score)
                setArmPosition(-600);

        }
        if (gamepad1.dpad_down && !(gamepad1.left_trigger > 0.5)) {
            if (specimen_score)
                setArmPosition(-440, 1);
            else
                setArmPosition(-2000);
        }
        if (gamepad1.dpad_right) {
            if (specimen_score)
                setArmPosition(-230);
        }
        if (gamepad1.share) {
            setArmPosition(-440, 0.1);
        }

        telemetry.addData("left trigger", gamepad1.left_trigger);
        if (gamepad1.left_trigger > 0.5) {
            int position = Arm.getCurrentPosition();
            if (gamepad1.dpad_up)
                setArmPosition(position - 5, 0.1);
            if (gamepad1.dpad_down)
                setArmPosition(position + 5, 0.1);
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