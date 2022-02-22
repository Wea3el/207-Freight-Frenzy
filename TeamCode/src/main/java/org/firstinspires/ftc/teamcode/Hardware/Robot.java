package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot
{
    //subsystem components
    public Lift lift;
    public DuckSpinner duck;
    public DriveTrain drive;
    final GamePadEx gp1, gp2;
    final HardwareMap hardwareMap;
    final Telemetry telemetry;
    final boolean isRed, isAuton;

    public Robot(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry, boolean isAuton, boolean isRed) {
        gp1 = new GamePadEx(gamepad1);
        gp2 = new GamePadEx(gamepad2);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.isAuton = isAuton;
        this.isRed = isRed;
    }

    public void init() {
        lift = new Lift(hardwareMap, telemetry);
        duck = new DuckSpinner(telemetry, isRed, hardwareMap);
        drive = new DriveTrain(hardwareMap, telemetry, isAuton);
    }

    public void updateTeleopState() {
        lift.updateTeleopState(gp1, gp2);
        duck.updateTeleopState(gp1, gp2);
        drive.updateTeleopState(gp1, gp2);
    }

    public void updateState() {
        lift.updateState();
        duck.updateState();
        if(isAuton){
            drive.updateState();
        }

    }

    public enum detectionState {
     TOP, MIDDLE,BOT;
    }
}
