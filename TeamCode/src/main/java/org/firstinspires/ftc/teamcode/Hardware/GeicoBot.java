package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;

import java.io.File;
import java.util.List;

public class GeicoBot
{
    public DcMotor frontLeft, backLeft, frontRight, backRight, intake, duck, lift;

    public Servo spit;

    public double power, strafePower, speed;

    HardwareMap map;
    Telemetry tele;

    public ElapsedTime runtime = new ElapsedTime();

    public BNO055IMU imu;
    public BNO055IMU.Parameters parameters;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Cube",
            "Marker"
    };
    final double     HEADING_THRESHOLD       = 5;

    // Additional Gyro device
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final String VUFORIA_KEY =
            "Acigj+7/////AAABmQugQe1ZQUopr3xiuORZW/1dG/ikPkZLNbC+QPCu21Rzj31nDeEOYFET8+CME7vkvCuEivLG3UEE7YDlkmimZWdyaxfOU2irEatGpMI6l5Pav7W9omY4k2gQcvOA47ml11AyUBomtPu/WUqbJUaviOswXkLMiHt7B7azcyuj/BI2b8Wdc3edXpraMkE++0sZdxn2cPH5lA2A+8CEhw4ogvpwjLiVVVxyQ5QYb6zN5FBVM1E/hpoEEgPCs89h6Plfn3AjbXu+1TEYKTLVgqEvdTlSpbni+5QOqcBHp7m9lXEfjF1OkIqOf1daD11VT0sCa/ZmTaZfgMUKa0p0NuTuCCJH1QwObC0oJFOkrd+h1PZU";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    public void init(HardwareMap map, Telemetry tele)
    {

        this.map = map;
        this.tele = tele;

        frontLeft = map.get(DcMotor.class, "FL");
        frontRight  = map.get(DcMotor.class, "FR");
        backLeft= map.get(DcMotor.class, "BL");
        backRight = map.get(DcMotor.class, "BR");
        intake = map.get(DcMotor.class, "intake");
        duck = map.get(DcMotor.class, "duck");
        lift = map.get(DcMotor.class, "lift");

        spit = map.get(Servo.class, "spit");

        imu = map.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        imu = map.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        tele.addData("Status", "Initialized");
    }

    public int autonDrive(MovementEnum movement, int target) {
        int x = -1;
        switch (movement) {
            case FORWARD:
                frontLeft.setTargetPosition(target);
                frontRight.setTargetPosition(target);
                backLeft.setTargetPosition(target);
                backRight.setTargetPosition(target);
                x = Math.max(backRight.getCurrentPosition(), Math.max(backLeft.getCurrentPosition(), Math.max(frontRight.getCurrentPosition(), frontLeft.getCurrentPosition())));
                break;

            case BACKWARD:
                frontLeft.setTargetPosition(-target);
                frontRight.setTargetPosition(-target);
                backLeft.setTargetPosition(-target);
                backRight.setTargetPosition(-target);
                x = Math.max(backRight.getCurrentPosition(), Math.max(backLeft.getCurrentPosition(), Math.max(-frontRight.getCurrentPosition(), -frontLeft.getCurrentPosition())));
                break;

            case LEFTSTRAFE:
                frontLeft.setTargetPosition(-target);
                frontRight.setTargetPosition(target);
                backLeft.setTargetPosition(target);
                backRight.setTargetPosition(-target);
                x = Math.max(backRight.getCurrentPosition(), Math.max(backLeft.getCurrentPosition(), Math.max(frontRight.getCurrentPosition(), frontLeft.getCurrentPosition())));
                break;

            case RIGHTSTRAFE:
                frontLeft.setTargetPosition(target);
                frontRight.setTargetPosition(-target);
                backLeft.setTargetPosition(-target);
                backRight.setTargetPosition(target);
                x = Math.max(-backRight.getCurrentPosition(), Math.max(-backLeft.getCurrentPosition(), Math.max(-frontRight.getCurrentPosition(), -frontLeft.getCurrentPosition())));
                break;

            case LEFTTURN:
                frontLeft.setTargetPosition(-target);
                frontRight.setTargetPosition(target);
                backLeft.setTargetPosition(-target);
                backRight.setTargetPosition(target);
                x = Math.max(backRight.getCurrentPosition(), Math.max(-backLeft.getCurrentPosition(), Math.max(frontRight.getCurrentPosition(), -frontLeft.getCurrentPosition())));
                break;

            case RIGHTTURN:
                frontLeft.setTargetPosition(target);
                frontRight.setTargetPosition(-target);
                backLeft.setTargetPosition(target);
                backRight.setTargetPosition(-target);
                x = Math.max(-backRight.getCurrentPosition(), Math.max(backLeft.getCurrentPosition(), Math.max(-frontRight.getCurrentPosition(), frontLeft.getCurrentPosition())));
                break;

            case STOP:
                frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
                frontRight.setTargetPosition(frontRight.getCurrentPosition());
                backLeft.setTargetPosition(backLeft.getCurrentPosition());
                backRight.setTargetPosition(backRight.getCurrentPosition());
                break;
        }
        return x;
    }
    public void resetEncoder(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        runtime.reset();
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
    private void lift(int targetPos, double speed)
    {

        int pos = targetPos;


        lift.setTargetPosition(pos);




        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        lift.setPower(speed);


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
            steer = gyroCorrect(angle, 1, actual , 0.05, 0.2);
            rightSpeed  = steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        frontLeft.setPower(leftSpeed);
        backLeft.setPower(leftSpeed);
        frontRight.setPower(rightSpeed);
        backRight.setPower(rightSpeed);


        // Display it for the driver.

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
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = map.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", map.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

}


