package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// for internal imu
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


//@Autonomous
public class Zach_Auto_Base extends OpMode {

    DcMotor left = null;
    DcMotor right = null;
    DcMotor left_intake = null;
    DcMotor right_intake = null;
    Servo left_servo = null;
    BNO055IMU imu = null;
    Orientation angles = null;
    ColorSensor colorSensor = null;
    ElapsedTime et = null;

    double elapsed_time = 0.0;
    double encoder_start_l = 0.0;
    double encoder_change = 0.0;
    double heading_start = 0.0;  // value at beginning of turning stage
    double heading_change = 0.0;  // change in angle since started to turn
    double heading = 0.0;
    double color_ratio = 1.0;
    double last_color_ratio = 1.0;

    long stage = -1;  // determines current behavior or robot

    long turn_direction = 1;
    long target_color = 1; // 1 is red, 2 is blue
    double initial_delay = 17.5;
    double stop_2 = 1800.0;
    boolean timeInit = false;

    public void init(){

        // setting up the internal imu
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
        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        encoder_start_l = left.getCurrentPosition();

        telemetry.addData("Status","running");
        telemetry.update();
    }

    public void loop() {

        if (!timeInit) {
            et = new ElapsedTime();
            timeInit = true;
        }

        elapsed_time = et.milliseconds() / 1000.0;  // convert to seconds
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        encoder_change = left.getCurrentPosition() - encoder_start_l;
        heading = angles.firstAngle;
        heading_change = heading - heading_start;

        // measure color ratio to detect target_color
        if (target_color == 1) { // red ratio
            color_ratio = colorSensor.red() / (0.5 * (colorSensor.green() + colorSensor.blue()));
        } else {                // blue ratio
            color_ratio = colorSensor.blue() / (0.5 * (colorSensor.green() + colorSensor.red()));
        }

        // correct for the jump in angle change at 180/-180
        if (heading_change < -180){
            heading_change = heading_change + 360;
        } else if (heading_change > 180){
            heading_change = heading_change - 360;
        }

        // determine if we are finished with a stage, if so "promote"
        if (stage == -1 & elapsed_time > initial_delay & elapsed_time < initial_delay + 2.0){
            stage = 1;
            encoder_start_l = left.getCurrentPosition();
        } else if (stage == 1 & encoder_change > 287){
            stage = 2;
            heading_start = heading;
        } else if (stage == 2 & turn_direction * heading_change < -84){
            stage = 3;
            encoder_start_l = left.getCurrentPosition();
        //} else if (stage == 3 & last_color_ratio > 1.2){
        } else if (stage == 3 & encoder_change > stop_2){
            stage = -1;
        }

        //  autonomous behavior determined by stage
        if (stage == -1) {         // stopped
            left.setPower(0.0);
            right.setPower(0.0);
        } else if (stage == 1) {   // go straight
            left.setPower(.6);
            right.setPower(.6);
        } else if (stage == 2) {   // turn right (if turn_direction = 1)
            left.setPower(turn_direction * 0.25);
            right.setPower(turn_direction * -0.25);
        } else if (stage == 3){      // go straight
            left.setPower(.2);
            right.setPower(.2);
        }

        telemetry.addData("Et (sec)", "%f", elapsed_time);
        telemetry.addData("Stage"," %d", stage);
        telemetry.addData("Heading"," %.2f", heading);
        telemetry.addData("heading Change:", "%.2f",  heading_change);
        telemetry.addData("encoder Change:", "%.2f",  encoder_change);
        telemetry.addData("encoder_start_l:", "%.2f",  encoder_start_l);
        telemetry.addData("color_ratio:", "%.2f",  color_ratio);
    }
}
