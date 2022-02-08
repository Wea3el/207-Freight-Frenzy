package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OldLift {
    private DcMotor slideLeft;
    private DcMotor slideRight;
    private Servo gateIn;
    private Servo gateOut;
    private Servo slope;

    //constructor
    public OldLift(HardwareMap map) {
        slideLeft = map.get(DcMotor.class, "SL");
        slideRight = map.get(DcMotor.class, "SR");
        gateIn = map.get(Servo.class, "GI");
        gateOut = map.get(Servo.class, "GO");
        slope = map.get(Servo.class, "S");
        //set position, reverse one lift motor's direction

        slideLeft.setDirection(DcMotor.Direction.REVERSE);
        slideRight.setDirection(DcMotor.Direction.FORWARD);

        gateIn.setPosition(0);
        gateOut.setPosition(0);
        slope.setPosition(0);
    }

    public void slide(double power) {
        slideLeft.setPower(1);
        slideRight.setPower(1);
    }
        public void GateInClose () {
            gateIn.setPosition(1);
        }

        public void GateInOpen () {
        gateIn.setPosition(0);
        }

        public void GateOutOpen () {
        gateOut.setPosition (0);
        }

        public void GateOutClose () {
        gateOut.setPosition (1);
        }

        // servo gateOut is 0.5 when closed and either 0 or 1 when open
}