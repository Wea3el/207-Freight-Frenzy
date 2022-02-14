package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name = "BlueDuckSub", group = "Testing")
public class BlueDuck extends OpMode {
    Robot robot;
    Robot.detectionState cameraDetect;
    public AutonState state;

    enum AutonState {
        PULLOUT, TURNDUCK, DRIVEDUCK, SPINDUCK, // ONE THING
        STRAFETOWER, DRIVEDEPOSIT, DEPOSIT, // ONETHING
        STRAFEWALL,

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

    }


    @Override
    public void loop() {
        switch(state){
            case PULLOUT:
                robot.drive.setTargetAndMove(10000, DriveTrain.Direction.NORTH );
                if (robot.drive.getState() == DriveTrain.DriveTrainState.IDLE) {
                    state = AutonState.TURNDUCK;
                }
                break;
            case TURNDUCK:
                robot.drive.onHeading(90);
                if(robot.drive.getState() == DriveTrain.DriveTrainState.IDLE){
                    state = AutonState.DRIVEDUCK;
                }

                break;
            case DRIVEDUCK:
                break;
            case SPINDUCK:
                break;
            case STRAFETOWER:
                break;
            case DRIVEDEPOSIT:
                break;
            case DEPOSIT:
                break;
            case STRAFEWALL:
                break;
        }

        telemetry.addLine(robot.drive.getState().name());
        telemetry.addLine(robot.lift.getLevel().name());
        telemetry.addLine(this.state.toString());
        telemetry.update();

    }

    @Override
    public void stop() {

    }



}