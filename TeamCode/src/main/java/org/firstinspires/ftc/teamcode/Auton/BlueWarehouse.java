package org.firstinspires.ftc.teamcode.Auton;

import android.view.FocusFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.DuckSpinner;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.VisionWrapper;


@Autonomous(name = "BlueWarehouseSub", group = "Testing")
public class BlueWarehouse extends OpMode {
    Robot robot;
    Lift.Level detectedLevel = Lift.Level.MID;
    public ElapsedTime runtime = new ElapsedTime();
    public AutonState state;
    public int target;
    private VisionWrapper vision;
    private int one, two, three;



    enum AutonState {
        MOVEFORTURN,TURN90, TURN180,STRAFETOWER, MOVETOWER, DEPOSIT, TURNWAREHOUSE, STRAFELINEUP, MOVEWAREHOUSE,END

    }

    @Override
    public void init() {
        robot = new Robot(gamepad1, gamepad2, hardwareMap, telemetry, true, false);
        robot.init();
        state = AutonState.MOVEFORTURN;
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
        robot.drive.setTargetAndMove(100, DriveTrain.Direction.BACKWARD,0.5);

    }


    @Override
    public void loop() {
        switch(state){
            case MOVEFORTURN:
                if(robot.drive.readyForNext()){
                    state = AutonState.TURN180;
                    runtime.reset();
                    robot.drive.waitAuton();
                }
                break;

            case TURN180:
                if(robot.drive.readyForNext() && runtime.milliseconds() > 3000){
                    runtime.reset();
                    state = AutonState.STRAFETOWER;
                    robot.drive.waitAuton();
                    robot.drive.setTargetAndMove(300, DriveTrain.Direction.LEFT, 0.5);
                    robot.lift.setStateLevel(Lift.States.MOVE, detectedLevel);
                }
                else{
                    robot.drive.turn(180);
                }
                break;
            case STRAFETOWER:
                if (robot.drive.readyForNext()) {
                    state = AutonState.MOVETOWER;
                    runtime.reset();
                    robot.drive.waitAuton();
                    if(detectedLevel == Lift.Level.BOTTOM){
                        target = 1000;
                    }
                    else if(detectedLevel == Lift.Level.MID){
                        target = 400;
                    }
                    else if(detectedLevel == Lift.Level.TOP){
                        target = 200;
                    }
                    robot.drive.setTargetAndMove(target, DriveTrain.Direction.BACKWARD,0.5);
                }
                break;

            case MOVETOWER:
                if (robot.drive.readyForNext()){
                    state = AutonState.DEPOSIT;
                    runtime.reset();
                    robot.drive.waitAuton();
                }

                break;
            case DEPOSIT:
                if(robot.lift.getState() == Lift.States.MOVE ){
                    state = AutonState.TURNWAREHOUSE;
                }
                else{
                    if(robot.lift.getState() == Lift.States.ATLEVEL){
                        robot.lift.dump();
                    }
                }
                break;
            case TURNWAREHOUSE:
                if(robot.drive.readyForNext() &&  runtime.milliseconds() >3000){
                    runtime.reset();
                    state = AutonState.STRAFELINEUP;
                    robot.drive.waitAuton();

                    if(detectedLevel == Lift.Level.BOTTOM){
                        target = 0;
                    }
                    else if(detectedLevel == Lift.Level.MID){
                        target = 200;
                    }
                    else if(detectedLevel == Lift.Level.TOP){
                        target = 400;
                    }
                    robot.drive.setTargetAndMove(target, DriveTrain.Direction.RIGHT,0.5);
                }

                else{
                    robot.drive.turn(90);
                }
                break;
            case STRAFELINEUP:
                if(robot.drive.readyForNext()){
                    runtime.reset();
                    state = AutonState.MOVEWAREHOUSE;
                    robot.drive.waitAuton();
                    robot.drive.setTargetAndMove(1000, DriveTrain.Direction.FORWARD,1);
                }
                break;
            case MOVEWAREHOUSE:
                if(robot.drive.readyForNext()){
                    runtime.reset();
                    state = AutonState.END;
                    robot.drive.waitAuton();
                    robot.drive.setTargetAndMove(0, DriveTrain.Direction.FORWARD, 0);
                }
                break;
            case END:
                robot.drive.setTargetAndMove(0, DriveTrain.Direction.FORWARD, 0);
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