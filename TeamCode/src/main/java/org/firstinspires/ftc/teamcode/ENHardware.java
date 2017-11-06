package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ENHardware
{

    final static double JEWEL_MIN_RANGE  = 0;
    final static double JEWEL_MAX_RANGE  = 1;
    final static double GLYPH_MIN_RANGE  = 0;
    final static double GLYPH_MAX_RANGE  = 1;



    double jewelPosition;
    double glyphPosition;

    double jewelDelta = 0.1;
    double glyphDelta = 0.1;

    public DcMotor strafe    = null;
    public DcMotor rightSideFront    = null;
    public DcMotor rightSideBack    = null;
    public DcMotor leftSideFront    = null;
    public DcMotor leftSideBack    = null;
    public 	Servo jewel = null;
    public Servo glyph = null;
    public Servo color_sensor = null;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public ENHardware(){

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;


        strafe = hwMap.dcMotor.get("STR");
        rightSideFront = hwMap.dcMotor.get("MRF");
        rightSideBack = hwMap.dcMotor.get("MRB");
        leftSideFront = hwMap.dcMotor.get("MLF");
        leftSideBack = hwMap.dcMotor.get("MLB");
        //ColorSensor = hwMap.servo.get("sensor_color");
        rightSideFront.setDirection(DcMotor.Direction.FORWARD);
        rightSideBack.setDirection(DcMotor.Direction.REVERSE);
        leftSideFront.setDirection(DcMotor.Direction.REVERSE);
        leftSideBack.setDirection(DcMotor.Direction.FORWARD);
        strafe.setDirection(DcMotor.Direction.FORWARD);

        jewel = hwMap.servo.get("JWL");
        glyph = hwMap.servo.get("GLY");

        jewelPosition = 1;
        glyphPosition = 0.1;


        rightSideFront.setPower(0);
        leftSideFront.setPower(0);
        leftSideBack.setPower(0);
        rightSideBack.setPower(0);
        strafe.setPower(0);


        leftSideFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSideFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSideBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSideBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafe.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

