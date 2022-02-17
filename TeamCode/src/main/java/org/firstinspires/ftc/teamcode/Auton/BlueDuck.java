package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.DuckSpinner;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name = "BlueDuckSub", group = "Testing")
public class BlueDuck extends OpMode {
    Robot robot;
    Robot.detectionState cameraDetect;
    public ElapsedTime runtime = new ElapsedTime();
    public AutonState state;

    enum AutonState {
        PULLOUT, TURNDUCK, DRIVEDUCK, SPINDUCK, // ONE THING
        STRAFETOWER, DRIVEDEPOSIT, DEPOSIT, // ONETHING
        BACKWALL,

    }

    @Override
    public void init() {
        robot = new Robot(gamepad1, gamepad2, hardwareMap, telemetry, true, false);
        robot.init();
        state = AutonState.PULLOUT;
    }

    @Override
    public void init_loop() {
        cameraDetect = Robot.detectionState.TOP;
    }

    @Override
    public void start() {
        robot.drive.setTargetAndMove(100, DriveTrain.Direction.BACKWARD);

    }


    @Override
    public void loop() {
        switch(state){
            case PULLOUT:
                if (robot.drive.getState() == DriveTrain.DriveTrainState.IDLE) {
                    state = AutonState.TURNDUCK;
                    runtime.reset();
                    robot.drive.waitAuton();
                }else {

                }

                break;
            case TURNDUCK:
                if(robot.drive.getState() == DriveTrain.DriveTrainState.IDLE && runtime.milliseconds() >3000){
                    runtime.reset();
                    state = AutonState.DRIVEDUCK;
                    robot.drive.setTargetAndMove(700, DriveTrain.Direction.BACKWARD);
                }else{
                    robot.drive.turn(90);
                }

                break;
            case DRIVEDUCK:
                if (robot.drive.getState() == DriveTrain.DriveTrainState.IDLE) {
                    runtime.reset();
                    state = AutonState.SPINDUCK;
                    robot.drive.waitAuton();
                }else {

                }


                break;
            case SPINDUCK:
                if(runtime.milliseconds() > 4000){
                    robot.duck.duckStates = DuckSpinner.States.STOP;
                    state = AutonState.STRAFETOWER;
                    robot.drive.waitAuton();
                    robot.drive.setTargetAndMove(1000, DriveTrain.Direction.RIGHT);
                }else {
                    robot.duck.duckStates = DuckSpinner.States.SPINBLUE;
                }

                break;
            case STRAFETOWER:
                if (robot.drive.getState() == DriveTrain.DriveTrainState.IDLE) {
                    state = AutonState.DRIVEDEPOSIT;
                    robot.drive.setTargetAndMove(1000, DriveTrain.Direction.FORWARD);
                }else {

                    robot.lift.setStateLevel(Lift.States.MOVE, Lift.Level.TOP);
                }

                break;
            case DRIVEDEPOSIT:
                if (robot.drive.getState() == DriveTrain.DriveTrainState.IDLE) {
                    state = AutonState.DEPOSIT;
                    robot.drive.waitAuton();
                }else {

                }

                break;
            case DEPOSIT:
                if(robot.lift.getState() == Lift.States.INTAKE ){
                    state = AutonState.BACKWALL;
                    robot.drive.setTargetAndMove(1000, DriveTrain.Direction.BACKWARD);
                }else {
                    if(robot.lift.getState() == Lift.States.ATLEVEL){
                        robot.lift.dump();
                    }
                }

                break;
            case BACKWALL:
                if(robot.drive.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.drive.waitAuton();
                }else{

                }
                break;
        }
        robot.updateState();

        telemetry.addLine(robot.drive.getState().name());
        telemetry.addLine(robot.lift.getLevel().name());
        telemetry.addLine(this.state.toString());
        telemetry.addLine(robot.drive.direction.toString());
        telemetry.addLine(robot.lift.getState().name());
        telemetry.addData("mode", robot.drive.FR.getMode());
        telemetry.addData("ticks FR", robot.drive.FR.getCurrentPosition());
        telemetry.addData("ticks BR", robot.drive.BR.getCurrentPosition());
        telemetry.addData("ticks BL" , robot.drive.FL.getCurrentPosition());
        telemetry.addData("ticks FL" , robot.drive.BL.getCurrentPosition());
        telemetry.addData("gyro", robot.drive.angle());
        telemetry.addData("lift", robot.lift.liftTicks());

        telemetry.addData("target", robot.drive.target);
        telemetry.update();

    }

    @Override
    public void stop() {

    }



}