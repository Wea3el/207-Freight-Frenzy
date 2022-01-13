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

//----------------------------
// WIFI PASSWORD IS PAGNOM207
//----------------------------

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpenCV.VisionWrapper;

@TeleOp(name="NewBotBlue", group="Iterative Opmode")
//@Disabled
public class NewBotBlue extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor intake;

    // duck
    private DcMotor duck;

    private double power;
    private double strafePower;
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

    int liftPos;

    private VisionWrapper visionWrapper;

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
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        gateIn = hardwareMap.get(Servo.class, "gateIn");
        slope = hardwareMap.get(Servo.class, "slope");
        gateOut = hardwareMap.get(Servo.class, "gateOut");

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        duck.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER );

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        visionWrapper = new VisionWrapper();
        visionWrapper.init(hardwareMap);

        slope.setPosition(0.2);
        liftPos = 0;

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
        if(gamepad1.right_trigger > 0.1){
            strafePower = gamepad1.right_trigger;
            frontRight.setPower(strafePower);
            frontLeft.setPower(-strafePower);
            backRight.setPower(-strafePower);
            backLeft.setPower(strafePower);
        }

        else if(gamepad1.left_trigger > 0.1){
            strafePower = gamepad1.left_trigger;
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
//            leftLift.setPower(gamepad2.right_stick_y *0.5);
//            rightLift.setPower(gamepad2.right_stick_y *0.5);

//            double liftpow = gamepad2.right_stick_y;
//            leftLift.setPower(liftpow);
//            rightLift.setPower(liftpow); //ROB: DOING THIS MIGHT HELP W SLACK ISSUE

            if(gamepad2.dpad_down && liftPos != 0)
            {
                //leftLift
            }

            if(gamepad2.left_stick_y == 0f)
            {
                // default position aka pp down

                gateIn.setPosition(0.33);
                slope.setPosition(0.2);
            }
            else if(gamepad2.left_stick_y > 0.1f)
            {


                // pp up position

                gateIn.setPosition(1);
                slope.setPosition(0.7);
            }

            if(gamepad2.b)
            {
                gateOut.setPosition(1);

            }
            else
            {
                gateOut.setPosition(0.5);
            }
        }

        if(gamepad2.x){
            power = 0.25;
            runtime.reset();
        }
        else if(gamepad2.y){
            power = -0.25;
            runtime.reset();
        }
        else{
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














        telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight", frontRight.getCurrentPosition());
        telemetry.addData("backRight", backRight.getCurrentPosition());
        telemetry.addData("backLeft", backLeft.getCurrentPosition());
        telemetry.addData("lift", rightLift.getCurrentPosition());
        telemetry.addData("duck", duck.getCurrentPosition());
        telemetry.addData("SPEED", power);

        telemetry.addData("BIG CAMERA", visionWrapper.currentDetermination());

        telemetry.addData("gateIn", gateIn.getPosition());
        telemetry.addData("gateOut", gateOut.getPosition());
        telemetry.addData("slope", slope.getPosition());

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





