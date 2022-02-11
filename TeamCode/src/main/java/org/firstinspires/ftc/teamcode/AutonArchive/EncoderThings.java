package org.firstinspires.ftc.teamcode.AutonArchive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//@Disabled
@Autonomous(name = "EncoderThings", group = "Testing")
public class EncoderThings extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor intake;
    private DcMotor duck; // duck
    private DcMotor lift;
    private double power;
    private double strafePower;
    private Servo spit;
    private double speed;
    final double     HEADING_THRESHOLD       = 3;      // As tight as we can make it with an integer gyro
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    boolean calib;
    int auto = 0;

    private int pos;

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
        lift = hardwareMap.get(DcMotor.class, "lift");
        spit = hardwareMap.get(Servo.class, "spit");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );


        pos = 0;

        imu.initialize(parameters);
    }

    private void drive(int targetPos, double speed)
    {

        int pos = cmToTicks(targetPos);


        frontLeft.setTargetPosition(pos);
        backLeft.setTargetPosition(pos);
        frontRight.setTargetPosition(pos);
        backRight.setTargetPosition(pos);



        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);


    }
    private void strafe(int targetPos, double speed, int direction)//true == 1, false == left
    {

        int pos = cmToTicks(targetPos) * direction;

        frontLeft.setTargetPosition(pos);
        backLeft.setTargetPosition(-pos);
        frontRight.setTargetPosition(-pos);
        backRight.setTargetPosition(pos);





        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);


    }

    @Override
    public void init_loop() {
        telemetry.addData("Gyro Calib?", imu.isGyroCalibrated());
        telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

    }

    @Override
    public void start() {
        telemetry.clear();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );


    }


    @Override
    public void loop() {


    switch(auto){
            case 0:

                int target = cmToTicks(7);
                drive(target, 0.5);


                if(frontLeft.getCurrentPosition()> target ){
                    auto++;

                }

                break;


            case(1):
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
                auto++;
                runtime.reset();
                break;

            case(2):
                target = cmToTicks(3.2);
                strafe(target, 0.3, -1);


                if(Math.abs(frontLeft.getCurrentPosition())> target ){
                    auto++;

                }
                runtime.reset();
                break;

            case(3):
                duck.setPower(0.3);
                if(runtime.milliseconds()>6000){
                    duck.setPower(0);
                    auto++;
                    runtime.reset();
                }

                break;
            case(4):
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
                auto++;
                runtime.reset();
            case(5):
                drive(cmToTicks(3), 1);


                if(Math.abs(frontLeft.getCurrentPosition())> cmToTicks(3) ){
                    auto++;

                }
                break;


                case(6):

                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
                onHeading(90);
                if(onHeading(90) & runtime.milliseconds()>2500) {
                    auto++;
                    }
                break;
            case(7):
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
                auto++;
                runtime.reset();
                break;
            case(8):
                target = cmToTicks(4.3);
                strafe(target, 0.5, -1);


                if(Math.abs(frontLeft.getCurrentPosition())> target ){
                    auto++;

                }
                runtime.reset();
                break;
            case(9):
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
                auto++;
                runtime.reset();
                break;

            case(10):

                drive(5000, 1);


                if(Math.abs(frontLeft.getCurrentPosition())> 5000 ){
                    auto++;

                }
                break;
            case(11):
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);







        }





        telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight", frontRight.getCurrentPosition());
        telemetry.addData("backRight", backRight.getCurrentPosition());
        telemetry.addData("backLeft", backLeft.getCurrentPosition());
        telemetry.addData("Status", imu.getSystemStatus().toShortString());
        telemetry.addData("Calib Status", imu.getCalibrationStatus().toString());
        telemetry.addData("Gyro Calib?", imu.isGyroCalibrated());
        telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("case", auto);
        telemetry.addData("time", runtime.milliseconds());
        telemetry.addData("cmToTicks(150)", cmToTicks(150));

        telemetry.update();


    }

    @Override
    public void stop() {

    }
    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @return
     */
    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @return
     */
    boolean onHeading(double angle) {
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
            steer = gyroCorrect(angle, 1, actual , 0.05, 0.3);
            rightSpeed  = steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        frontLeft.setPower(leftSpeed);
        backLeft.setPower(leftSpeed);
        frontRight.setPower(rightSpeed);
        backRight.setPower(rightSpeed);


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
    public int cmToTicks(double cm){
        int ticksPerRev = 537;
        int diameter = 10;
        return (int) (ticksPerRev *(cm  / (diameter* Math.PI)));
    }

}


