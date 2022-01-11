package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveTrain {
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor BR;
    private DcMotor BL;
    private BNO055IMU gyro;

    public DriveTrain(HardwareMap map) {

        FR  = map.get(DcMotor.class, "FR");
        FL = map.get(DcMotor.class, "FL");
        BR = map.get(DcMotor.class, "BR");
        BL = map.get(DcMotor.class, "BL");

        gyro = map.get(BNO055IMU.class, "gyro");
        //set position, reverse one lift motor's direction

        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER );

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void tankDrive()
    {

    }

    private void drive(int targetPos, double speed)
    {
//
//        int pos = cmToTicks(targetPos);
//
//
//        frontLeft.setTargetPosition(pos);
//        backLeft.setTargetPosition(pos);
//        frontRight.setTargetPosition(pos);
//        backRight.setTargetPosition(pos);
//
//
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        frontLeft.setPower(speed);
//        backLeft.setPower(speed);
//        frontRight.setPower(speed);
//        backRight.setPower(speed);


    }

    public void forward(int cm, int speed)
    {
        FL.setTargetPosition(cmToTicks(cm));
        FR.setTargetPosition(cmToTicks(cm));
        BL.setTargetPosition(cmToTicks(cm));
        BR.setTargetPosition(cmToTicks(cm));

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(speed);
        FR.setPower(speed);
        BL.setPower(speed);
        BR.setPower(speed);
    }

    public void backwards(int cm)
    {

    }

    public void strafeleft(int cm)
    {

    }

    public void strafeRight(int cm)
    {

    }

    public int cmToTicks(double cm)
    {
        int ticksPerRev = 537;
        int diameter = 10;
        return (int) (ticksPerRev *(cm  / (diameter* Math.PI)));
    }
}
