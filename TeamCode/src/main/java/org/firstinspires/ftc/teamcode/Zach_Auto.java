package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Zach_Auto extends OpMode {

    DcMotor left;
    DcMotor right;
    DcMotor left_intake = null;
    DcMotor right_intake = null;
    Servo left_servo = null;


    private ElapsedTime et = null;
    boolean halt = false;
    double left_trigger = 0.0;
    double elapsed_time = 0.0;
    double power = 0.2;
    double rotation_direction = 1.0;
    double encoder_start_l = 0.0;
    double encoder_change = 0.0;
    double gyro_heading_start = 0.0;
    double gyro_heading_change = 0.0;
    double gyro_heading = 0.0;
    double last_gyro_heading = 0.0;
    double gyro_heading_change_at_transition = 0.0;
    double headingLastNonZeroDelta = 0.0;
    long stage = 1;
    long num_cycles = 0;
    long gyroNumHeadingChanges = 0;
    private ModernRoboticsI2cGyro gyro = null;
    private int waitForStartTime = 0;

    public void init(){

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        left.setDirection(DcMotor.Direction.REVERSE);
        left_intake = hardwareMap.dcMotor.get("left_intake");
        right_intake = hardwareMap.dcMotor.get("right_intake");
        left_servo = hardwareMap.servo.get("left_servo");
        et = new ElapsedTime();
        encoder_start_l = left.getCurrentPosition();
     //   gyro = hardwareMap.

        telemetry.addData("Status","running");
        telemetry.update();
    }

    public void loop() {

        elapsed_time = et.milliseconds() / 1000.0;
        telemetry.addData("Et (sec)", "%f", elapsed_time);

        last_gyro_heading = gyro_heading;
        gyro_heading = gyro.getHeading();
        encoder_change = left.getCurrentPosition() - encoder_start_l;
        gyro_heading_change = gyro_heading - gyro_heading_start;

        if (gyro_heading_change < -180){
            gyro_heading_change = gyro_heading_change + 360;
        } else if (gyro_heading_change > 180){
            gyro_heading_change = gyro_heading_change - 360;
        }

        if (stage == 1 & encoder_change > 1361){
            stage = 2;
            gyro_heading_start = left.getCurrentPosition();
        } else if (stage == 2 & gyro_heading_change < -90){
            stage = 3;
            encoder_start_l = left.getCurrentPosition();
            gyro_heading_change_at_transition = gyro_heading_change;  //logging
        } else if (stage == 3 & encoder_change > 4083){
            stage = -1;
        }

        //      auto stuff
        if (stage == -1) {
            left.setPower(0.0);
            right.setPower(0.0);
        } else if (stage == 1) {
            left.setPower(.5);
            right.setPower(.5);
        } else if (stage == 2) {
            left.setPower(.5);
            right.setPower(-.5);
        } else if (stage == 3);{
            left.setPower(.5);
            right.setPower(.5);
            }


//        if (elapsed_time < 3.0) {
//            // go forward
//            right.setPower(power);
//            left.setPower(power);
//            telemetry.addData("Mode", "%s", "1 - Forward1");
//
//        } else if (elapsed_time < 5.25) {
//            // turn left
//            right.setPower(rotation_direction * power);
//            left.setPower(-rotation_direction * power);
//            telemetry.addData("Mode", "%s", "2 - Turn");
//
//        } else if (elapsed_time < 10.25) {
//
//            right.setPower(power);
//            left.setPower(power);
//            telemetry.addData("Mode", "%s", "3 - Forward");
//
//        } else {
//            right.setPower(0.0);
//            left.setPower(0.0);
//            telemetry.addData("Mode", "%s", "4 - Finished/Stopped");
//        }
    }
}
