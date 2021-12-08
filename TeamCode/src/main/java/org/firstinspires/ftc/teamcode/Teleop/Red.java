/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Red", group="Iterative Opmode")
//@Disabled
public class Red extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor intake;
    private DcMotor duck;
    private DcMotor lift;
    private double power;
    private double strafePower;
    private Servo spit;
    private double speed = 1;
    private double curHeading;
    private boolean fieldCentric = false;
    private double thing;
    private double thing1;
    ModernRoboticsI2cRangeSensor rangeSensor;

    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        frontRight  = hardwareMap.get(DcMotor.class, "FR");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        backLeft= hardwareMap.get(DcMotor.class, "BL");
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        intake = hardwareMap.get(DcMotor.class, "intake");
        duck = hardwareMap.get(DcMotor.class, "duck");
        lift = hardwareMap.get(DcMotor.class, "lift");
        spit = hardwareMap.get(Servo.class, "spit");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distance");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        duck.setMode(DcMotor.RunMode.RUN_USING_ENCODER );

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        if(fieldCentric){
            curHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            // Setup a variable for each drive wheel to save power level for telemetry

            double forward = (-1) * gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double clockwise = gamepad1.right_stick_x;

            // Apply the turn modifier
            clockwise *= 1; //Test and see if should be lowered / raised for desired control

            // Convert to Radians for Math.sin/cos
            double orient = Math.toRadians(curHeading);
            double sin = Math.sin(orient);
            double cos = Math.cos(orient);

            // Apply the rotation matrix
            double temp = forward * cos - strafe * sin;
            strafe = forward * sin + strafe * cos;
            forward = temp;

            // Set power values
            double flPow = forward + clockwise + strafe;
            double frPow = forward - clockwise - strafe;
            double blPow = forward + clockwise - strafe;
            double rrPow = forward - clockwise + strafe;

            double max = Math.max(1, Math.abs(forward) + Math.abs(strafe) + Math.abs(clockwise));

            // Clip power values to within acceptable ranges for the motors
            flPow /= max;
            frPow /= max;
            blPow /= max;
            rrPow /= max;

            frontLeft.setPower(flPow);
            frontRight.setPower(frPow);
            backLeft.setPower(blPow);
            backRight.setPower(rrPow);
        }
        else{
            if(gamepad1.left_trigger > 0.1){
                strafePower = gamepad1.left_trigger;
                frontRight.setPower(strafePower);
                frontLeft.setPower(-strafePower);
                backRight.setPower(-strafePower);
                backLeft.setPower(strafePower);
            }

            else if(gamepad1.right_trigger > 0.1){
                strafePower = gamepad1.right_trigger;
                frontRight.setPower(-strafePower);
                frontLeft.setPower(strafePower);
                backRight.setPower(strafePower);
                backLeft.setPower(-strafePower);
            }
            else{
                frontRight.setPower(gamepad1.right_stick_y * speed);
                frontLeft.setPower(gamepad1.left_stick_y * speed);
                backRight.setPower(gamepad1.right_stick_y * speed );
                backLeft.setPower(gamepad1.left_stick_y * speed);
                intake.setPower(gamepad2.left_stick_y);
                lift.setPower(gamepad2.right_stick_y *0.75);

            }


            if(gamepad2.b){
                spit.setPosition(1);
            }

            else{
                spit.setPosition(0);
            }


            if(gamepad2.x){
                power = 0.275;
                runtime.reset();
            }
            else if(gamepad2.y){
                power = -0.275;
                runtime.reset();
            }
            else if(runtime.milliseconds() >1250){
                power = 0;
            }
            duck.setPower(power);



            if(gamepad1.right_stick_button ){
                if(speed == 1){
                    speed = 0.5;
                }
                else{
                    speed = 1;
                }
            }


        }

        if(gamepad1.b){
            fieldCentric = !fieldCentric;

        }















        telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight", frontRight.getCurrentPosition());
        telemetry.addData("backRight", backRight.getCurrentPosition());
        telemetry.addData("backLeft", backLeft.getCurrentPosition());
        telemetry.addData("lift", lift.getCurrentPosition());
        telemetry.addData("spit", spit.getPosition());
        telemetry.addData("SPEED", duck.getPower());
        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

        // Send calculated power to wheels

        // Show the elapsed game time and wheel power.

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
