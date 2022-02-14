package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveTrain extends Subsystem {
    //auton stuff
    private int target = 0;
    private double angle = 0;
    private double strafePower = 0;
    //region Physical Components
    private DcMotor FL, FR, BL, BR;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private ElapsedTime runtime = new ElapsedTime();
    final double     HEADING_THRESHOLD       = 5;


    //region Movement Stats
    private DriveTrainState state;
    private Direction direction;
    private double drivePower;
    //endregion

    //region Dependent Classes
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    //endregion

    public DriveTrain(HardwareMap hmap, Telemetry tele, boolean isAuton) { // REMOVE THE ISAUTON BOOLEAN
        super(tele);

        // Initialise dependency classes
        hardwareMap = hmap;
        telemetry = tele;

        // Initialise states
        state = DriveTrainState.IDLE;

        // Initialize motor names
        FR  = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL= hardwareMap.get(DcMotor.class, "BL");
        FL = hardwareMap.get(DcMotor.class, "FL");

        // Set motor direction according to their orientation on the bot
        //   motors on the left side will be reversed so that their directions coorespond to
        //      the motors on the right
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        initImu();

        if (isAuton) { // Set the motors to brake for ONLY auton
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        else{
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addData("Drive Train", "initialized");
    }

    @Override
    public void updateState() {
        switch(state){

            case MOVE:
                drivePower = 0.5;
                moveMotorsWithDir(direction);
                if(Math.abs(BR.getCurrentPosition()) > target){
                    runtime.reset();
                    state = DriveTrainState.STOPPING;
                }


                break;
            case TURN:
                if (FL.getMode().equals(DcMotor.RunMode.STOP_AND_RESET_ENCODER)) {
                    FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if(onHeading(angle) & runtime.milliseconds()>1500){
                    state = DriveTrainState.STOPPING;
                }
                break;
            case STOPPING:
                stop();
                setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if(runtime.milliseconds() > 500){
                    state = DriveTrainState.IDLE;
                }

                break;
            case IDLE:
                runtime.reset();
                break;
        }

    }

    @Override
    public void updateTeleopState(GamePadEx drivingGP, GamePadEx OtherGP) {
        if (FL.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        double axisRightY = drivingGP.getAxis(GamePadEx.ControllerAxis.RIGHT_Y);
        double axisLeftY = drivingGP.getAxis(GamePadEx.ControllerAxis.LEFT_Y);
        telemetry.addData("Right Y", axisRightY);
        telemetry.addData("Left Y", axisLeftY);
        if (drivingGP.getAxis(GamePadEx.ControllerAxis.LEFT_TRIGGER) > 0.1) { // Strafe left
            strafePower = drivingGP.getAxis(GamePadEx.ControllerAxis.LEFT_TRIGGER);
            FR.setPower(-strafePower);
            FL.setPower(strafePower);
            BR.setPower(strafePower);
            BL.setPower(-strafePower);
        } else if (drivingGP.getControl(GamePadEx.ControllerButton.RTRIGGER)) { // Strafe right
            strafePower = drivingGP.getAxis(GamePadEx.ControllerAxis.RIGHT_TRIGGER);
            FR.setPower(strafePower);
            FL.setPower(-strafePower);
            BR.setPower(-strafePower);
            BL.setPower(strafePower);
        }
        else{
            BR.setPower(axisLeftY);
            FR.setPower(axisLeftY);
            BL.setPower(axisRightY);
            FL.setPower(axisRightY);

        }



    }

    public DriveTrainState getState() {
        return state;
    }

    public enum DriveTrainState {
        MOVE, // Auton, escapable
        TURN, // Auton, escapable
        STOPPING, // Auton, escapable
        IDLE, // Reset encoders Auton, escapable
    }

    public enum Direction {
        NORTH, // | (/\)
        NORTHEAST, // / (/\)
        EAST, // - (>)
        SOUTHEAST, // \ (\/)
        SOUTH, // | (\/)
        SOUTHWEST, // / (\/)
        WEST, // - (<)
        NORTHWEST, // \ (/\)
        NONE, // Fallback or no movement
    }

    public void moveMotorsWithDir(Direction dir) {
        if (dir == Direction.NORTH) {
            BR.setPower(drivePower);
            FR.setPower(drivePower);
            BL.setPower(drivePower);
            FL.setPower(drivePower);
        } else if (dir == Direction.NORTHEAST) {
            BR.setPower(drivePower);
            FR.setPower(0.0);
            BL.setPower(0.0);
            FL.setPower(drivePower);
        } else if (dir == Direction.EAST) {
            BR.setPower(drivePower);
            FR.setPower(-drivePower);
            BL.setPower(-drivePower);
            FL.setPower(drivePower);
        } else if (dir == Direction.SOUTHEAST) {
            BR.setPower(0.0);
            FR.setPower(-drivePower);
            BL.setPower(-drivePower);
            FL.setPower(0.0);
        } else if (dir == Direction.SOUTH) {
            BR.setPower(-drivePower);
            FR.setPower(-drivePower);
            BL.setPower(-drivePower);
            FL.setPower(-drivePower);
        } else if (dir == Direction.SOUTHWEST) {
            BR.setPower(-drivePower);
            FR.setPower(0.0);
            BL.setPower(0.0);
            FL.setPower(-drivePower);
        } else if (dir == Direction.WEST) {
            BR.setPower(-drivePower);
            FR.setPower(drivePower);
            BL.setPower(drivePower);
            FL.setPower(-drivePower);
        } else if (dir == Direction.NORTHWEST) {
            BR.setPower(0.0);
            FR.setPower(drivePower);
            BL.setPower(drivePower);
            FL.setPower(0.0);
        }
    }

    public void setMotorMode(DcMotor.RunMode mode) {
        BR.setMode(mode);
        FR.setMode(mode);
        BL.setMode(mode);
        FL.setMode(mode);
    }

    // Sets the motor power of all the drive motors to 0
    @Override
    public void stop() {
        BR.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        FL.setPower(0.0);
    }

    public void initImu() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        imu.initialize(parameters);
    }
    public void setTargetAndMove(int t, Direction d){
        target = t;
        direction = d;
        FR.setTargetPosition(target);
        BR.setTargetPosition(target);
        FL.setTargetPosition(target);
        BL.setTargetPosition(target);
        this.state = DriveTrainState.MOVE;
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public boolean onHeading(double angle) {
        this.state = DriveTrainState.TURN;
        double actual = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;
        double error =  (angle - actual)-180;

        // determine turn power based on +/- error


        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;

        }
        else {
            steer = gyroCorrect(angle, 1, actual , 0.05, 0.2);
            rightSpeed  = steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        FL.setPower(leftSpeed);
        BL.setPower(leftSpeed);
        FR.setPower(rightSpeed);
        BR.setPower(rightSpeed);


        // Display it for the driver.
        telemetry.addData("error", error);
        telemetry.addData("speed", steer);
        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */


    /**
     * @param gyroTarget The target heading in degrees, between 0 and 360
     * @param gyroRange  The acceptable range off target in degrees, usually 1 or 2
     * @param gyroActual The current heading in degrees, between 0 and 360
     * @param minSpeed   The minimum power to apply in order to turn (e.g. 0.05 when moving or 0.15 when stopped)
     * @param addSpeed   The maximum additional speed to apply in order to turn (proportional component), e.g. 0.3
     */

    public double gyroCorrect(double gyroTarget, double gyroRange, double gyroActual, double minSpeed, double addSpeed) {
        double delta = (gyroTarget - gyroActual + 360.0) % 360.0; //the difference between target and actual mod 360
        if (delta > 180.0) delta -= 360.0; //makes delta between -180 and 180
        if (Math.abs(delta) > gyroRange) { //checks if delta is out of range
            double gyroMod = delta / 45.0; //scale from -1 to 1 if delta is less than 45 degrees
            if (Math.abs(gyroMod) > 1.0)
                gyroMod = Math.signum(gyroMod); //set gyromod to 1 or -1 if the error is more than 45 degrees
            return (minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod);
        } else {
            return 0.0;
        }
    }

}