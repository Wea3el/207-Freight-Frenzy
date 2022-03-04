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
    public int target = 0;
    private double angle = 0;
    private double strafePower = 0;
    private double driveSpeed;
    //region Physical Components
    public DcMotor FL, FR, BL, BR;
    public BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private ElapsedTime runtime = new ElapsedTime();
    final double     HEADING_THRESHOLD       = 5;


    //region Movement Stats
    private DriveTrainState state;
    public Direction direction;
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
        state = DriveTrainState.WAIT;

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
                moveMotorsWithDir(Direction.FORWARD, driveSpeed);
                if(Math.abs(Math.abs(BR.getCurrentPosition()) - Math.abs(target)) < 5){
                    runtime.reset();
                    state = DriveTrainState.IDLE;
                }
                break;

            case TURN:
                if(onHeading(angle) || runtime.milliseconds() >4000){
                    state = DriveTrainState.IDLE;
                    this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                break;

            case IDLE:
                stop();
                setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case WAIT:
                stop();
                setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
        }

    }

    @Override
    public void updateTeleopState(GamePadEx drivingGP, GamePadEx OtherGP) {
        if (FL.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        double axisRightY = drivingGP.getAxis(GamePadEx.ControllerAxis.LEFT_Y);
        double axisLeftY = drivingGP.getAxis(GamePadEx.ControllerAxis.RIGHT_Y);


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
        WAIT, //Auton, does nothing
        IDLE, // Reset encoders Auton, escapable

    }

    public boolean readyForNext(){
        return this.state  == DriveTrainState.IDLE && FR.getCurrentPosition() == 0 && BR.getCurrentPosition() == 0 && FL.getCurrentPosition()== 0 && BL.getCurrentPosition() == 0;
    }

    public enum Direction {
        FORWARD, // | (/\)
        NORTHEAST, // / (/\)
        RIGHT, // - (>)
        SOUTHEAST, // \ (\/)
        BACKWARD, // | (\/)
        SOUTHWEST, // / (\/)
        LEFT, // - (<)
        NORTHWEST, // \ (/\)
        NONE, // Fallback or no movement
    }

    public void moveMotorsWithDir(Direction dir, double drivePower) {
        if (dir == Direction.FORWARD) {
            BR.setPower(drivePower);
            FR.setPower(drivePower);
            BL.setPower(drivePower);
            FL.setPower(drivePower);
        } else if (dir == Direction.NORTHEAST) {
            BR.setPower(drivePower);
            FR.setPower(0.0);
            BL.setPower(0.0);
            FL.setPower(drivePower);
        } else if (dir == Direction.RIGHT) {
            BR.setPower(drivePower);
            FR.setPower(-drivePower);
            BL.setPower(-drivePower);
            FL.setPower(drivePower);
        } else if (dir == Direction.SOUTHEAST) {
            BR.setPower(0.0);
            FR.setPower(-drivePower);
            BL.setPower(-drivePower);
            FL.setPower(0.0);
        } else if (dir == Direction.BACKWARD) {
            BR.setPower(-drivePower);
            FR.setPower(-drivePower);
            BL.setPower(-drivePower);
            FL.setPower(-drivePower);
        } else if (dir == Direction.SOUTHWEST) {
            BR.setPower(-drivePower);
            FR.setPower(0.0);
            BL.setPower(0.0);
            FL.setPower(-drivePower);
        } else if (dir == Direction.LEFT) {
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
    public void setTargetAndMove(int t, Direction d, double speed){
        target = t;
        direction = d;
        driveSpeed = speed;
        if (d == Direction.FORWARD) {
            BR.setTargetPosition(t);
            FR.setTargetPosition(t);
            BL.setTargetPosition(t);
            FL.setTargetPosition(t);
        } else if (d == Direction.NORTHEAST) {
            BR.setTargetPosition(t);
            FR.setTargetPosition(0);
            BL.setTargetPosition(0);
            FL.setTargetPosition(t);
        } else if (d == Direction.RIGHT) {
            BR.setTargetPosition(t);
            FR.setTargetPosition(-t);
            BL.setTargetPosition(-t);
            FL.setTargetPosition(t);
        } else if (d == Direction.SOUTHEAST) {
            BR.setTargetPosition(0);
            FR.setTargetPosition(-t);
            BL.setTargetPosition(-t);
            FL.setTargetPosition(0);
        } else if (d == Direction.BACKWARD) {
            BR.setTargetPosition(-t);
            FR.setTargetPosition(-t);
            BL.setTargetPosition(-t);
            FL.setTargetPosition(-t);
        } else if (d == Direction.SOUTHWEST) {
            BR.setTargetPosition(-t);
            FR.setTargetPosition(0);
            BL.setTargetPosition(0);
            FL.setTargetPosition(-t);
        } else if (d == Direction.LEFT) {
            BR.setTargetPosition(-t);
            FR.setTargetPosition(t);
            BL.setTargetPosition(t);
            FL.setTargetPosition(-t);
        } else if (d == Direction.NORTHWEST) {
            BR.setTargetPosition(0);
            FR.setTargetPosition(t);
            BL.setTargetPosition(t);
            FL.setTargetPosition(0);
        }

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.state = DriveTrainState.MOVE;

    }

    public void waitAuton(){
        this.state = DriveTrainState.WAIT;

    }


    public double angle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
    }

    public void turn(double angle2){
        angle = angle2;
        state = DriveTrainState.TURN;
        this.stop();
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


//    public boolean onHeading(double angle) {
//        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double actual = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
//        boolean  onTarget = false ;
//        double leftSpeed;
//        double rightSpeed;
//        double error = angle - actual;
//        double driveScale = error * .005;
//
//        if (driveScale > 1) {
//            driveScale = 1;
//        } else if (driveScale < -1) {
//            driveScale = -1;
//        }
//
//        leftSpeed = 0 + driveScale;
//        rightSpeed = 0 - driveScale;
//
//        FL.setPower(leftSpeed);
//        BL.setPower(leftSpeed);
//        FR.setPower(rightSpeed);
//        BR.setPower(rightSpeed);
//        if(Math.abs(error) < 1){
//            onTarget = true;
//            FL.setPower(0);
//            BL.setPower(0);
//            FR.setPower(0);
//            BR.setPower(0);
//        }
////        if(angle >= 180){
////            float currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
////
////            // to convert to a 0-360 scale, if the current heading is negative add
////            //    360 to it
////            currHeading = currHeading < 0 ? 360 + currHeading : currHeading;
////
////            // difference between target and current heading
////            double difference = angle - currHeading;
////            telemetry.addData("Difference is ", difference);
////
////            // If the bot is within 1 degree of the target, stop the bot and return true
////            if (Math.abs(difference) <= 2) {
////                telemetry.addLine("Stopping the bot");
////                onTarget = true;
////            }
////
////
////            // If the bot is within a 30 degree threshold of the target, slow it down to 25% of the desired speed to prevent overshooting
////            else if (Math.abs(difference) <= 30) {
////                FR.setPower(-0.1);
////                BR.setPower(-0.1);
////                FL.setPower(0.1);
////                BL.setPower(0.1);
////            } else { // Otherwise use normal speed
////                FR.setPower(-0.5);
////                BR.setPower(-0.5);
////                FL.setPower(0.5);
////                BL.setPower(0.5);
////            }
////        }
////        else{
////            double error =  (angle - actual)-180;
////
////            // determine turn power based on +/- error
////
////
////            if (Math.abs(error) <= HEADING_THRESHOLD ) {
////                steer = 0.0;
////                leftSpeed  = 0.0;
////                rightSpeed = 0.0;
////                onTarget = true;
////
////            }
////            else {
////                steer = gyroCorrect(angle, 1, actual , 0.05, 0.2);
////                rightSpeed  = steer;
////                leftSpeed   = -rightSpeed;
////
////            }
////
////            // Send desired speeds to motors.
////            FL.setPower(leftSpeed);
////            BL.setPower(leftSpeed);
////            FR.setPower(rightSpeed);
////            BR.setPower(rightSpeed);
////
////
////            // Display it for the driver.
////            telemetry.addData("error", error);
////            telemetry.addData("speed", steer);
////        }
//
//        return onTarget;
//    }
    public boolean onHeading(double target) {
        double actual = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        double delta = (target - ((int) actual) + 360.0) % 360.0; //the difference between target and actual mod 360
        if (delta > 180.0) delta -= 360.0; //makes delta between -180 and 180
        if (Math.abs(delta) >= 1.0) { //checks if delta is out of range
            double gyroMod = delta / 45.0; //scale from -1 to 1 if delta is less than 45 degrees
            if (Math.abs(gyroMod) > 1.0) {
                gyroMod = Math.signum(gyroMod); //set gyromod to 1 or -1 if the error is more than 45 degrees
            }
            this.rotateBot(0.05 * Math.signum(gyroMod) + 0.1 * gyroMod);
            return false;
        } else {
            this.rotateBot(0.0);
            return true;
        }
    }

    private void rotateBot(double pow) {
        FL.setPower(-pow);
        FR.setPower(pow);
        BL.setPower(-pow);
        BR.setPower(pow);
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