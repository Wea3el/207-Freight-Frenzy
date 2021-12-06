package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import java.io.File;
import java.util.List;

@Autonomous(name = "RedEncoderThings", group = "Testing")
public class RedEncoderThings extends OpMode {
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
    final double     HEADING_THRESHOLD       = 5;      // As tight as we can make it with an integer gyro
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    boolean calib;
    int auto = 0;
    int place;
    boolean detected = false;
    private int pos;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Cube",
            "Marker",
            "Ball"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
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
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER );


        pos = 0;

        imu.initialize(parameters);
        initVuforia();
        initTfod();
        spit.setPosition(0);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        tfod.activate();

        // The TensorFlow software will scale the input images from the camera to a lower resolution.
        // This can result in lower detection accuracy at longer distances (> 55cm or 22").
        // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
        // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
        // should be set to the value of the images used to create the TensorFlow Object Detection model
        // (typically 16/9).
        tfod.setZoom(1, 16.0/9.0);
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
    private void strafe(int targetPos, double speed, int direction)//1 = right, -1 == left
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
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());

                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                    if(recognition.getLeft() > 10 & recognition.getRight() >150 &recognition.getLeft() <300){
                        place = 1;
                        telemetry.addLine("done");
                        detected = true;
                        break;
                    }
                    else if(recognition.getLeft() > 400 & recognition.getRight() >800 ){
                        place = 2;
                        telemetry.addLine("done");
                        detected = true;
                        break;
                    }
                    else if(!detected){
                        place = 3;
                        telemetry.addLine("done");
                        break;

                    }


                }

                telemetry.update();
            }
        }


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


                drive(cmToTicks(6.5), 0.3);


                if(frontLeft.getCurrentPosition() > cmToTicks(6.5) ){
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    auto++;

                }
                runtime.reset();

                break;


            case(1):


                if(runtime.milliseconds() > 500){
                    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
                    auto++;
                }


                break;

            case(2):
                strafe(950, 0.3, -1);


                if(Math.abs(frontLeft.getCurrentPosition())> 950 ){
                    auto++;
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);

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
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);

                }

                break;


            case(6):

                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );

                if(onHeading(90) & runtime.milliseconds()>2500) {
                    auto++;
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
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

                strafe(1325, 0.5, -1);


                if(Math.abs(frontLeft.getCurrentPosition())> 1300 ){
                    auto++;
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);

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
                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );

                if(onHeading(90) & runtime.milliseconds()>1000) {
                    auto++;
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                }
                break;
            case(11):
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
                auto++;
                runtime.reset();
                break;
            case(12):
                int target = 0;
                if(place ==1 ){
                    target = 230;

                }
                else if(place == 2){
                    target = 800;
                }
                else{
                    target = 1200;

                }
                lift(target,0.3);
                if(runtime.milliseconds()> 2000){
                    auto++;
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);

                }

                break;

            case(13):
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
                auto++;
                runtime.reset();
                break;
            case(14):
                target = 0;
                if(place ==1 ){
                    target = cmToTicks(62);
                }
                else if(place == 2){
                    target = cmToTicks(60);
                }
                else{
                    target = cmToTicks(59);

                }

                drive(target, 0.4);


                if(Math.abs(frontLeft.getCurrentPosition())> target ){
                    auto++;
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    spit.setPosition(0.7);

                }
                runtime.reset();
                break;



            case(15):
                if(runtime.milliseconds() >1000){
                    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
                    auto++;
                    runtime.reset();
                }

                break;

            case(16):
                drive(-100,0.2);


                if(cmToTicks(-100)>Math.abs(frontLeft.getCurrentPosition())){
                    auto++;
                    spit.setPosition(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);

                }
                runtime.reset();
                break;
            case(17):
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );

                auto++;
                runtime.reset();
                break;

            case(18):
                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );

                if(onHeading(90) & runtime.milliseconds()>1000) {
                    auto++;
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                }
                break;
            case(19):
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );

                auto++;
                runtime.reset();
                break;
            case(20):
                strafe(200,0.2, 1);
                break;
            case(21):
                lift.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);






        }





        telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight", frontRight.getCurrentPosition());
        telemetry.addData("backRight", backRight.getCurrentPosition());
        telemetry.addData("backLeft", backLeft.getCurrentPosition());
        telemetry.addData("lift", lift.getCurrentPosition());
        telemetry.addData("Status", imu.getSystemStatus().toShortString());
        telemetry.addData("Calib Status", imu.getCalibrationStatus().toString());
        telemetry.addData("Gyro Calib?", imu.isGyroCalibrated());
        telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("case", auto);
        telemetry.addData("time", runtime.milliseconds());
        telemetry.addData("cmToTicks(150)", cmToTicks(150));
        telemetry.addData("place", place);




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
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.1f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

}