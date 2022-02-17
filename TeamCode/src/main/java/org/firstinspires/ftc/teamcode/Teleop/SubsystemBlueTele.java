package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.DuckSpinner;
import org.firstinspires.ftc.teamcode.Hardware.GamePadEx;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
@TeleOp(name="SubsystemBlue", group="Iterative Opmode")
public class SubsystemBlueTele extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(gamepad1, gamepad2, hardwareMap, telemetry, false, false);
        robot.init();
    }

    @Override
    public void loop() {
       robot.updateTeleopState();
       robot.updateState();
        telemetry.addLine(robot.lift.getState().name());
        telemetry.addLine(robot.lift.getLevel().name());
        telemetry.update();

    }

}
