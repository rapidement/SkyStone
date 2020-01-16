package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Mattsfirstopmode extends OpMode {
    DcMotor left;
    DcMotor right;
    DcMotor left_intake = null;
    DcMotor right_intake = null;
    Servo left_servo = null;
    double left_trigger = 0.0;  // store gamepad position for intake to avoid duelling intake motors


    public void init(){
        left = hardwareMap.dcMotor.get("left");
        //left.setDirection(DcMotor.Direction.REVERSE);  // makes it easier to program
        right = hardwareMap.dcMotor.get("right");
        left_intake = hardwareMap.dcMotor.get("left_intake");
        right_intake = hardwareMap.dcMotor.get("right_intake");
        left_servo = hardwareMap.servo.get("left_servo");
        telemetry.addData("Status","running");
        telemetry.update();
    }

    public void loop(){

        // arcade drive
        right.setPower(-gamepad1.left_stick_y-gamepad1.left_stick_x);
        left.setPower(gamepad1.left_stick_y-gamepad1.left_stick_x);

        //Intake Motors
        left_trigger = gamepad1.left_trigger;
        left_intake.setPower(left_trigger);
        right_intake.setPower(-left_trigger);

        if (left_trigger == 0.0) {
            left_intake.setPower(-gamepad1.right_trigger);
            right_intake.setPower(gamepad1.right_trigger);
        }

        //hook
        if (gamepad1.y){
            left_servo.setPosition(left_servo.getPosition()+.02);
        } else if (gamepad1.a){
            left_servo.setPosition(left_servo.getPosition()-.02);
        }

        telemetry.addData("left.joystick.x",gamepad1.left_stick_x);
        telemetry.addData("left.joystick.y",gamepad1.left_stick_y);
        telemetry.update();
    }
}
