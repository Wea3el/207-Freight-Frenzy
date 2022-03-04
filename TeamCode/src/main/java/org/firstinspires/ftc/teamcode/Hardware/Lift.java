package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Lift extends Subsystem
{
    private DcMotor intake, lift;
    public Servo gateIn, gateOut, slope, capstone;
    private States state;
    private Level level;
    private ElapsedTime runtime = new ElapsedTime();
    private double intakeSpeed = 0;
    private double liftPower = 0.8;

    private RevColorSensorV3 color;
    private TouchSensor limit;


    public Lift(HardwareMap map, Telemetry telemetry) {
        super(telemetry);
        state = States.INTAKE;
        level = Level.TOP;

        lift = map.get(DcMotor.class, "lift");
        intake = map.get(DcMotor.class, "intake");

        gateIn = map.get(Servo.class, "gateIn");
        gateOut = map.get(Servo.class, "gateOut");
        slope = map.get(Servo.class, "slope");
        capstone = map.get(Servo.class, "capstone");

        slope.setPosition(0.6);

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        color = map.get(RevColorSensorV3.class, "color");
        limit = map.get(TouchSensor.class, "Limit");

        telemetry.addData("Lift Status", "Initialized");
    }

    @Override
    public void updateState() {
        switch(state) {

            case INTAKE:
                if(color.getDistance(DistanceUnit.CM) < 6){
                    state = States.IN;
                    runtime.reset();
                }
                else{
                    intake.setPower(intakeSpeed);
                    gateOut.setPosition(0.7);
                    gateIn.setPosition(-0.7);
                    slope.setPosition(0.6);
                }
                break;

            case IN:
                liftPower = 1;
                gateIn.setPosition(0.33);
                slope.setPosition(0.85);

                if(runtime.milliseconds() >1000){
                    intake.setPower(0);
                }
                else{
                    intake.setPower(-0.5);
                }
                break;

            case MOVE:
                if(setLiftPos(level.numTicks) )
                {
                    if(level == Level.INTAKE){
                        state = States.INTAKE;
                    }
                    else{
                        state = States.ATLEVEL;
                    }


                }
                if(level == Level.BOTTOM || level == Level.INTAKE){
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift.setPower(-liftPower);
                } else {
                    lift.setPower(liftPower);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION );
                }


                break;

            case ATLEVEL:
                runtime.reset();
                break;

            case DUMP:
                gateOut.setPosition(0.3);
                if(runtime.milliseconds()>3000){
                    slope.setPosition(0.6);
                    gateOut.setPosition(0.7);
                    level = Level.INTAKE;
                    state = States.MOVE;
                }
                break;
            case MANUAL:
                break;
        }


    }

    @Override
    public void updateTeleopState(GamePadEx gp1, GamePadEx gp2)
    {
        switch(state) {
            case INTAKE:
                liftPower = 0;
                level = Level.TOP;
                this.intakeSpeed = gp2.getAxis(GamePadEx.ControllerAxis.LEFT_Y);
                runtime.reset();
                if(gp1.getControlDown(GamePadEx.ControllerButton.B)){
                    state = States.IN;
                }
                break;

            case IN:
                liftPower = 1;
                if(gp2.getControlDown(GamePadEx.ControllerButton.B)){
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    state = States.MOVE;
                }
                break;

            case MOVE:
                
                break;

            case ATLEVEL:
                if(gp2.getControlDown(GamePadEx.ControllerButton.Y)){
                    state = States.DUMP;
                }

                break;

            case DUMP:
                break;

            case MANUAL:
                intake.setPower(gp2.getAxis(GamePadEx.ControllerAxis.LEFT_Y));
                double liftpow =   gp2.getAxis(GamePadEx.ControllerAxis.RIGHT_Y);
                if(limit.isPressed() && liftpow<0){
                    liftpow = 0;
                }
                lift.setPower(liftpow);
                if(gp2.getAxis(GamePadEx.ControllerAxis.LEFT_Y) == 0f)
                {
                    // default position aka pp down

                    gateIn.setPosition(0.33);
                    slope.setPosition(0.85);
                }
                else if(gp2.getAxis(GamePadEx.ControllerAxis.LEFT_Y)> 0.1f)
                {
                    // pp up position
                    gateIn.setPosition(-0.7);
                    slope.setPosition(0.6);
                }

                if(gp2.getControl(GamePadEx.ControllerButton.B))
                {
                    gateOut.setPosition(0.30);

                }
                else
                {
                    gateOut.setPosition(0.7);
                }
                break;

        }
        if(gp2.getControlDown(GamePadEx.ControllerButton.X)) {
            level = Level.TOP;
        }
        else if(gp2.getControlDown(GamePadEx.ControllerButton.A)) {
            level = Level.BOTTOM;
        }
        else if(gp2.getControlDown(GamePadEx.ControllerButton.GUIDE)){
            state = States.MANUAL;
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(gp1.getControl(GamePadEx.ControllerButton.B)){
            capstone.setPosition(1);
        }
        else{
            capstone.setPosition(0.6);
        }
        telemetry.addData("lift",lift.getCurrentPosition());
        telemetry.addData("limit", limit.isPressed());

    }
    public void dump(){
        runtime.reset();
        state = States.DUMP;
    }

    public int liftTicks(){
        return lift.getCurrentPosition();
    }

    @Override
    public void stop() {

    }

    public boolean setLiftPos(int numTicks)
    {
        lift.setTargetPosition(numTicks);
        if (level == Level.INTAKE || level == Level.BOTTOM) {
            if(limit.isPressed() || lift.getCurrentPosition() < 0){
                return true;
            }
        } else {
            if (Math.abs(lift.getCurrentPosition() - numTicks) <= 5) {
                lift.setPower(0.0);
                return true;
            }
        }

        return false;
    }
    public void setStateLevel(States state, Level level)
        {
            this.state = state;
            this.level = level;
        }
    public States getState() {
        return state;
    }

    public Level getLevel() {
        return level;
    }

    public enum States
    {
        INTAKE,
        MOVE,
        ATLEVEL,
        DUMP,
        IN,
        MANUAL,
    }

    public enum Level
    {
        TOP(3500),
        MID(2000),
        BOTTOM(30),
        INTAKE(100);

        public final int numTicks;

        private Level(int ticks)
        {
            this.numTicks = ticks;
        }

    }
}
