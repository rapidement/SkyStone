package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// for internal imu


@Autonomous
public class Zach_R_red extends Zach_Auto_Base {

    public void init(){
        super.init();
        turn_direction = 1;
        target_color = 1; // red
    }
}
