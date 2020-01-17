package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// for internal imu


@Autonomous
public class Zach_L_blue extends Zach_Auto_Base {

    public void init(){
        super.init();
        turn_direction = -1;
        target_color = 2;  // blue
    }
}
