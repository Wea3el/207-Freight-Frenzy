package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.DuckSpinner;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.VisionWrapper;

@Autonomous(name = "RedDuckSub", group = "Testing")
public class RedSide extends OpMode {
    Robot robot;
    Lift.Level detectedLevel = Lift.Level.MID;
    public ElapsedTime runtime = new ElapsedTime();
    public AutonState state;
    public int target;
    private VisionWrapper vision;
    private int one, two, three;



    enum AutonState {
        PULLOUT, TURNDUCK, DRIVEDUCK, SPINDUCK, // ONE THING
        STRAFETOWER, DRIVEDEPOSIT, DEPOSIT, // ONETHING
        BACKWALL, STRAFEPARK, END

    }

    @Override
    public void init() {
        robot = new Robot(gamepad1, gamepad2, hardwareMap, telemetry, true, true);
        robot.init();
        state = AutonState.PULLOUT;
        vision = new VisionWrapper(telemetry);
        vision.init(hardwareMap);
        detectedLevel = Lift.Level.TOP; // immediately overwritten but safer without null
        this.one = 0;
        this.two = 0;
        this.three = 0;
    }

    @Override
    public void init_loop() {
        // Get current detection every loop
        this.detectedLevel = this.vision.currentDetermination();

        if (this.detectedLevel != null) {
            // Add to value if detected
            switch (this.detectedLevel) {
                case TOP:
                    this.one++;
                    break;
                case MID:
                    this.two++;
                    break;
                case BOTTOM:
                    this.three++;
                    break;
            }


            telemetry.addData("Current detected level: ", this.detectedLevel);

            telemetry.addLine("-------------------------------------");
            telemetry.addLine("Overall detection numbers: (PRESS A TO RESET)");
            telemetry.addData("LEVEL 1: ", this.one);
            telemetry.addData("LEVEL 2: ", this.two);
            telemetry.addData("LEVEL 3: ", this.three);

            telemetry.update();
        }

    }

    @Override
    public void start() {
        robot.drive.setTargetAndMove(180, DriveTrain.Direction.BACKWARD, 0.5);


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
                if(robot.drive.readyForNext() &&  runtime.milliseconds() >3000){
                    runtime.reset();
                    state = AutonState.DRIVEDUCK;
                    robot.drive.setTargetAndMove(690, DriveTrain.Direction.BACKWARD,0.1);
                }else{
                    robot.drive.turn(270);
                }

                break;
            case DRIVEDUCK:
                if (robot.drive.getState() == DriveTrain.DriveTrainState.IDLE) {
                    runtime.reset();
                    state = AutonState.SPINDUCK;
                    robot.drive.waitAuton();
                }


                break;
            case SPINDUCK:
                if(runtime.milliseconds() > 4000){
                    robot.duck.duckStates = DuckSpinner.States.STOP;
                    state = AutonState.STRAFETOWER;
                    robot.drive.waitAuton();
                    robot.drive.setTargetAndMove(1750, DriveTrain.Direction.LEFT, 0.5);
                    robot.lift.setStateLevel(Lift.States.MOVE, detectedLevel);

                }else {
                    robot.duck.duckStates = DuckSpinner.States.SPIN;
                }

                break;
            case STRAFETOWER:
                if (robot.drive.readyForNext()) {
                    state = AutonState.DRIVEDEPOSIT;
                    robot.drive.waitAuton();
                    if(detectedLevel == Lift.Level.BOTTOM){
                        target = 1050;
                    }
                    else if(detectedLevel == Lift.Level.MID){
                        target = 700;
                    }
                    else if(detectedLevel == Lift.Level.TOP){
                        target = 500;
                    }
                    robot.drive.setTargetAndMove(target, DriveTrain.Direction.FORWARD,0.5);
                }

                break;
            case DRIVEDEPOSIT:
                if (robot.drive.getState() == DriveTrain.DriveTrainState.IDLE) {
                    state = AutonState.DEPOSIT;
                    robot.drive.waitAuton();
                }

                break;
            case DEPOSIT:
                if(robot.lift.getState() == Lift.States.MOVE ){
                    state = AutonState.BACKWALL;
                    if(detectedLevel == Lift.Level.BOTTOM){
                        robot.drive.setTargetAndMove(1300, DriveTrain.Direction.BACKWARD,0.5);
                    }
                    else{
                        robot.drive.setTargetAndMove(900, DriveTrain.Direction.BACKWARD,0.5);
                    }

                }else {
                    if(robot.lift.getState() == Lift.States.ATLEVEL){
                        robot.lift.dump();
                    }
                }

                break;
            case BACKWALL:
                if(robot.drive.readyForNext()){
                    robot.drive.waitAuton();
                    this.state = AutonState.STRAFEPARK;
                    robot.drive.setTargetAndMove(500, DriveTrain.Direction.RIGHT,0.5);
                }else{

                }
                break;
            case STRAFEPARK:
                if(robot.drive.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.drive.waitAuton();
                    this.state = AutonState.END;
                    robot.drive.setTargetAndMove(0, DriveTrain.Direction.LEFT, 0);

                }
                break;

            case END:
                robot.drive.setTargetAndMove(0, DriveTrain.Direction.LEFT, 0);
                robot.drive.stop();
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