package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.DuckSpinner;
import org.firstinspires.ftc.teamcode.Hardware.GamePadEx;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
@TeleOp(name="SubsystemBlue", group="Iterative Opmode")
public class SubsystemBlueTele extends OpMode {
    private DriveTrain driveTrain;
    private Lift lift;
    private DuckSpinner duckSpinner;
    Robot robot;



    @Override
    public void init() {
        driveTrain = new DriveTrain(hardwareMap, telemetry, false);
        lift = new Lift(hardwareMap, telemetry);
        duckSpinner = new DuckSpinner(telemetry, true, hardwareMap);
        robot = new Robot(gamepad1, gamepad2, hardwareMap, telemetry, false, false);
    }

    @Override
    public void loop() {
        driveTrain.updateTeleopState(new GamePadEx(gamepad1), new GamePadEx(gamepad2));
        lift.updateTeleopState(new GamePadEx(gamepad1), new GamePadEx(gamepad2));
        lift.updateState();
        duckSpinner.updateTeleopState(new GamePadEx(gamepad1), new GamePadEx(gamepad2));
        duckSpinner.updateState();
        telemetry.addLine(lift.getState().name());
        telemetry.addLine(lift.getLevel().name());
        telemetry.update();
    }

}
