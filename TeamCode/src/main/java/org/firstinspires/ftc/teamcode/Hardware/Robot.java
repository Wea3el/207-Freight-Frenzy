package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="RedRobotTeleOp", group="Iterative Opmode")
public class Robot extends OpMode
{
    //subsystem components
    Lift lift;
    DuckSpinner duck;
    DriveTrain drive;
    GamePadEx gp1, gp2;

    public Robot() {}

    @Override
    public void init() {
        gp1 = new GamePadEx(gamepad1);
        gp2 = new GamePadEx(gamepad2);
        lift = new Lift(hardwareMap, telemetry);
        duck = new DuckSpinner(telemetry, true, hardwareMap);
        drive = new DriveTrain(hardwareMap, telemetry, false);
    }

    @Override
    public void loop() {
        lift.updateTeleopState(gp1, gp2);
        duck.updateTeleopState(gp1, gp2);
        drive.updateTeleopState(gp1, gp2);
    }
}
