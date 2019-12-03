package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Mattsfirstopmode extends OpMode {
    DcMotor left;
    DcMotor right;

    public void init(){
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        telemetry.addData("Status","running");
        telemetry.update();
    }

    public void loop(){
        right.setPower(-gamepad1.left_stick_y-gamepad1.left_stick_x);
        left.setPower(gamepad1.left_stick_y-gamepad1.left_stick_x);
        //right.setPower(0.4);
        //left.setPower(-0.4);
        telemetry.addData("left.joystick.y",gamepad1.left_stick_y);
        telemetry.addData("right.joystick.y",gamepad1.right_stick_y);
        telemetry.update();
    }

}
