package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends Subsystem
{
    private DcMotor intake, leftLift, rightLift;
    private Servo gateIn, gateOut, slope;
    private States states;
    private Level level;

    public Lift(HardwareMap map, Telemetry telemetry) {
        super(telemetry);
        states = States.INTAKE;
        level = Level.INTAKE;

        leftLift = map.get(DcMotor.class, "leftLift");
        rightLift = map.get(DcMotor.class, "rightLift");
        intake = map.get(DcMotor.class, "intake");

        gateIn = map.get(Servo.class, "gateIn");
        gateOut = map.get(Servo.class, "gateOut");
        slope = map.get(Servo.class, "slope");

        slope.setPosition(0.2);

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER );

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Lift Status", "Initialized");
    }

    @Override
    public void updateState(Gamepad gp1, Gamepad gp2) {
        switch(states) {
            case INTAKE:
                double intakePower = gp2.left_stick_y;
                intake.setPower(intakePower); //gp2.left_stick_y sets intake power
                break;

            case MOVE:
                double liftPower = 0.5 * gp2.right_stick_y;
//                leftLift.setPower(liftPower);
//                rightLift.setPower(liftPower);

                if(setLiftPos(level.numTicks, liftPower))
                {
                    states = States.ATLEVEL;
                }
                break;

            case ATLEVEL:
                leftLift.setPower(0.0);
                rightLift.setPower(0.0);
                intake.setPower(0.0);
                break;

            case DUMP:

                break;

            case MANUAL:

                break;
        }
    }

    @Override
    public void updateTeleopState(Gamepad gp1, Gamepad gp2)
    {
//        if(gp2.x) // this might actually be gp2.y
//        {
//            duckStates = DuckSpinner.States.SPINBLUE;
//        }
//        else if(gp2.y) // this might actually be gp2.x
//        {
//            duckStates = DuckSpinner.States.SPINRED;
//        }
//        else
//        {
//            duckStates = DuckSpinner.States.STOP;
//        }
    }

    @Override
    public void stop() {

    }

    public boolean setLiftPos(int numTicks, double liftPower)
    {
        leftLift.setTargetPosition(numTicks);
        rightLift.setTargetPosition(numTicks);

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);

        if(Math.abs(leftLift.getCurrentPosition() - numTicks) <= 5)
        {
            leftLift.setPower(0.0);
            rightLift.setPower(0.0);

            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            return true;
        }

        return false;
    }

    enum States
    {
        INTAKE,
        MOVE,
        ATLEVEL,
        DUMP,
        MANUAL
    }

    enum Level
    {
        TOP(0, "Top"),
        MIDDLE(0, "Middle"),
        BOTTOM(0, "Bottom"),
        INTAKE(0, "Intake");

        private int numTicks;
        private String level;

        private Level(int ticks, String level)
        {
            numTicks = ticks;
            this.level = level;
        }

        public int getTargetTicks()
        {
            return this.numTicks;
        }

        public String getLevel()
        {
            return this.level;
        }
    }
}
