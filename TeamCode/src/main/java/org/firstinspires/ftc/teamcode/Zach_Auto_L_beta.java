package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@Autonomous
public class Zach_Auto_L_beta extends OpMode {

    DcMotor left;
    DcMotor right;
    DcMotor left_intake = null;
    DcMotor right_intake = null;
    Servo left_servo = null;
    BNO055IMU imu = null;
    Orientation angles = null;

    private ElapsedTime et = null;
    double elapsed_time = 0.0;
    double encoder_start_l = 0.0;
    double encoder_change = 0.0;

    double heading_start = 0.0;
    double heading_change = 0.0;
    double heading = 0.0;

    long stage = -1;

    public void init(){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        left.setDirection(DcMotor.Direction.REVERSE);
        left_intake = hardwareMap.dcMotor.get("left_intake");
        right_intake = hardwareMap.dcMotor.get("right_intake");
        left_servo = hardwareMap.servo.get("left_servo");
        et = new ElapsedTime();
        encoder_start_l = left.getCurrentPosition();

        telemetry.addData("Status","running");
        telemetry.update();
    }

    public void loop() {

        elapsed_time = et.milliseconds() / 1000.0;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;

        encoder_change = left.getCurrentPosition() - encoder_start_l;
        heading_change = heading - heading_start;

        if (heading_change < -180){
            heading_change = heading_change + 360;
        } else if (heading_change > 180){
            heading_change = heading_change - 360;
        }

       if (stage == -1 & elapsed_time > 20 & elapsed_time < 22 ) {
           stage = 1;
       } else if (stage == 1 & encoder_change > 1361){
            stage = 2;
            heading_start = heading;
       } else if (stage == 2 & heading_change < 84){
            stage = 3;
            encoder_start_l = left.getCurrentPosition();
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
            left.setPower(-.15);
            right.setPower(.15);
        } else if (stage == 3){
            left.setPower(.5);
            right.setPower(.5);
        }
        telemetry.addData("Et (sec)", "%f", elapsed_time);
        telemetry.addData("Stage"," %d", stage);
        telemetry.addData("Heading"," %.2f", heading);
        telemetry.addData("heading Change:", "%.2f",  heading_change);
        telemetry.addData("encoder Change:", "%.2f",  encoder_change);
        telemetry.addData("encoder_start_l:", "%.2f",  encoder_start_l);
    }
}
