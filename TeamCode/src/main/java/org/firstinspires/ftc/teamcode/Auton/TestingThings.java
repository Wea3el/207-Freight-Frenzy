package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.OpenCV.VisionWrapper;

import java.io.File;
import java.util.List;
//@Disabled
@Autonomous(name = "TestingThings", group = "Testing")
public class TestingThings extends OpMode{

    private double power;
    private double strafePower;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor intake;

    // duck
    private DcMotor duck;

    private double speed = 1;
    private boolean xHeld;
    private boolean yHeld;

    // lift motors
    private DcMotor leftLift;
    private DcMotor rightLift;

    // intake and outtake servos
    private Servo gateIn;
    private Servo slope;
    private Servo gateOut;

    private Servo cap;

    private VisionWrapper visionWrapper;

    final double     HEADING_THRESHOLD       = 5;      // As tight as we can make it with an integer gyro
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    boolean calib;
    int auto = 0;
    int place;
    boolean detected = false;
    private int pos;

    @Override
    public void init() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        telemetry.addLine("Ready.");
        telemetry.update();

        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        intake = hardwareMap.get(DcMotor.class, "intake");
        duck = hardwareMap.get(DcMotor.class, "duck");
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        gateIn = hardwareMap.get(Servo.class, "gateIn");
        slope = hardwareMap.get(Servo.class, "slope");
        gateOut = hardwareMap.get(Servo.class, "gateOut");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER );

        gateIn.setPosition(0.33);
        gateOut.setPosition(0.5);
        slope.setPosition(0.7);

        pos = 0;

        imu.initialize(parameters);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Gyro Calib?", imu.isGyroCalibrated());
        telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
    }

    @Override
    public void loop() {
        telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight", frontRight.getCurrentPosition());
        telemetry.addData("backRight", backRight.getCurrentPosition());
        telemetry.addData("backLeft", backLeft.getCurrentPosition());

        telemetry.addData("gateIn", gateIn.getPosition());
        telemetry.addData("gateOut", gateOut.getPosition());
        telemetry.addData("slope", slope.getPosition());

        telemetry.addData("Status", imu.getSystemStatus().toShortString());
        telemetry.addData("Calib Status", imu.getCalibrationStatus().toString());
        telemetry.addData("Gyro Calib?", imu.isGyroCalibrated());
        telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("case", auto);
        telemetry.addData("time", runtime.milliseconds());
        telemetry.addData("cmToTicks(150)", cmToTicks(150));
        telemetry.addData("place", place);
    }

    public int cmToTicks(double cm){
        int ticksPerRev = 537;
        int diameter = 10;
        return (int) (ticksPerRev *(cm  / (diameter* Math.PI)));
    }

}
