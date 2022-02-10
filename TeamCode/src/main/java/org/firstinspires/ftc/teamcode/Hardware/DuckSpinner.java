package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DuckSpinner extends Subsystem
{
    States duckStates;
    private final double duckSpeed;
    private DcMotor duck1, duck2;

    public DuckSpinner(Telemetry telemetry, boolean isRed, HardwareMap map) {
        super(telemetry);
        duck1  = map.get(DcMotor.class, "duck1");
        duck2  = map.get(DcMotor.class, "duck2");


        duckSpeed = 0.7 * (isRed ? 1 : -1);
        duckStates = States.STOP;
    }

    @Override
    public void updateState() {

        switch (duckStates)
        {
            case STOP:
                duck1.setPower(0);
                duck2.setPower(0);
                break;
            case SPINRED:
                duck1.setPower(duckSpeed);
                duck2.setPower(duckSpeed);
                break;
            case SPINBLUE:
                duck1.setPower(duckSpeed);
                duck2.setPower(duckSpeed);
                break;
        }
    }

    @Override
    public void updateTeleopState(GamePadEx gp1, GamePadEx gp2)
    {
        if(gp2.getControlDown(GamePadEx.ControllerButton.X)) // this might actually be gp2.y
        {
            duckStates = States.SPINBLUE;
        }
        else if(gp2.getControlDown(GamePadEx.ControllerButton.Y)) // this might actually be gp2.x
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
