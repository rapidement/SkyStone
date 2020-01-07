package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class DumbAut extends OpMode {

    DcMotor left;
    DcMotor right;
    DcMotor left_intake = null;
    DcMotor right_intake = null;
    Servo left_servo = null;

    double left_trigger = 0.0;
    private ElapsedTime et = null;
    double elapsed_time = 0.0;
    double power = 0.2;
    double rotation_direction = 1.0;

    public void init(){
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        left.setDirection(DcMotor.Direction.REVERSE);
        left_intake = hardwareMap.dcMotor.get("left_intake");
        right_intake = hardwareMap.dcMotor.get("right_intake");
        left_servo = hardwareMap.servo.get("left_servo");
        et = new ElapsedTime();

        telemetry.addData("Status","running");
        telemetry.update();
    }

    public void loop() {

        elapsed_time = et.milliseconds() / 1000.0;
        telemetry.addData("Et (sec)", "%f", elapsed_time);
        if (elapsed_time < 3.0) {
            // go forward
            right.setPower(power);
            left.setPower(power);
            telemetry.addData("Mode", "%s", "1 - Forward1");

        } else if (elapsed_time < 5.25) {
            // turn left
            right.setPower(rotation_direction * power);
            left.setPower(-rotation_direction * power);
            telemetry.addData("Mode", "%s", "2 - Turn");

        } else if (elapsed_time < 10.25) {

            right.setPower(power);
            left.setPower(power);
            telemetry.addData("Mode", "%s", "3 - Forward");

        } else {
            right.setPower(0.0);
            left.setPower(0.0);
            telemetry.addData("Mode", "%s", "4 - Finished/Stopped");
        }
    }
}
