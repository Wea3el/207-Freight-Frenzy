package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DuckSpinner extends Subsystem
{
    States duckStates;
    private final double duckSpeed;
    public DuckSpinner(Telemetry telemetry, boolean isRed) {
        super(telemetry);
        duckSpeed = 0.7 * (isRed ? 1 : -1);
        duckStates = States.STOP;
    }

    @Override
    public void updateState(HardwareMap map, Gamepad gp1, Gamepad gp2) {
        DcMotor duck = map.get(DcMotor.class, "duck");

        switch (duckStates)
        {
            case STOP:
                duck.setPower(0);
                break;
            case SPINRED:
                duck.setPower(duckSpeed);
                break;
            case SPINBLUE:
                duck.setPower(duckSpeed);
                break;
        }
    }

    @Override
    public void updateTeleopState(Gamepad gp1, Gamepad gp2)
    {
        if(gp2.x) // this might actually be gp2.y
        {
            duckStates = States.SPINBLUE;
        }
        else if(gp2.y) // this might actually be gp2.x
        {
            duckStates = States.SPINRED;
        }
        else
        {
            duckStates = States.STOP;
        }
    }

    @Override
    public void stop() {

    }

    enum States
    {
        STOP,
        SPINRED,
        SPINBLUE
    }
}
