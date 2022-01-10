package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DuckSpinner {

    private DcMotor duck;

    public DuckSpinner(HardwareMap map) {



        duck = map.get(DcMotor.class, "duck");
        duck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void right () {
        duck.setPower(-0.2);



    }
    public void left (){
        duck.setPower(0.2);
    }
}
