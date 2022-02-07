package org.firstinspires.ftc.teamcode.Hardware;

        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Gamepad;
        import com.qualcomm.robotcore.hardware.HardwareMap;

        import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends Subsystem
{
    States states;
    public Intake(Telemetry telemetry) {
        super(telemetry);
        // states = States.BOTTOM;
    }

    @Override
    public void updateState(HardwareMap map, Gamepad gp1, Gamepad gp2) {
        DcMotor leftLift = map.get(DcMotor.class, "leftLift");
        DcMotor rightLift = map.get(DcMotor.class, "rightLift");
        DcMotor intake = map.get(DcMotor.class, "intake");


        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        // DcMotor leftLift = map.get(DcMotor.class, "duck");

        switch (states)
        {
            case ATBOTTOM:
                intake.setPower(gp2.left_stick_y);
                break;
            case MOVETOBOTTOM:

                break;
            case ATTOP:
                break;
            case MOVETOTOP:
                break;
            case DUMP:
                break;
        }
    }

    @Override
    public void updateTeleopState(Gamepad gp1, Gamepad gp2)
    {
        if(gp2.x) // this might actually be gp2.y
        {
            // states = States.SPINBLUE;
        }
        else if(gp2.y) // this might actually be gp2.x
        {
            // states = States.SPINRED;
        }
        else
        {
            // states = States.STOP;
        }
    }

    @Override
    public void stop() {

    }

    enum Pos
    {
        TOP,
        MIDDLE,
        BOTTOM
    }

    enum States
    {
        ATBOTTOM,
        MOVETOTOP,
        ATTOP,
        DUMP,
        MOVETOBOTTOM,

    }
}
