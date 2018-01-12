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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 */

public class SeriousHardware
{
    public final static double jewelUp = 0;
    public final static double jewelDown = 0.65;
    public final static double glyphdumpUp = 0.1;
    public final static double glyphdumpDown = 1;
    public final static double glyleftUp = 0.4;
    public final static double glyleftDown = 0.96;
    public final static double glyrightUp = 0.6;
    public final static double glyrightDown = 0;
    public final static double glyStart = 0.55;

    //Servo claw
    //CrServo relicIO, relicUD

    public DcMotor rightSideFront, rightSideBack, leftSideFront, leftSideBack, strafe, glyphlift, glyphright, glyphleft = null;
    public Servo glyphdump, jewel, glyleft, glyright = null;
    public DeviceInterfaceModule cdim = null;
    public AdafruitI2cColorSensor sensorRGB = null;
    public OpticalDistanceSensor odsSensor = null;
    public BNO055IMU imu = null;
    public DigitalChannel digitalTouch = null;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public SeriousHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        strafe = hwMap.get(DcMotor.class, "STR");
        rightSideFront = hwMap.get(DcMotor.class, "MRF");
        rightSideBack = hwMap.get(DcMotor.class, "MRB");
        leftSideFront = hwMap.get(DcMotor.class, "MLF");
        leftSideBack = hwMap.get(DcMotor.class, "MLB");
        glyphlift = hwMap.get(DcMotor.class, "LFT");
        glyphleft = hwMap.get(DcMotor.class, "GLYL");
        glyphright = hwMap.get(DcMotor.class, "GLYR");

        jewel = hwMap.get(Servo.class, "JWL");
        glyphdump = hwMap.get(Servo.class, "GLY");
        glyleft = hwMap.get(Servo.class, "GLYSL");
        glyright = hwMap.get(Servo.class, "GLYSR");



        sensorRGB = (AdafruitI2cColorSensor) hwMap.get("sensor_color");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        imu = (BNO055IMU) hwMap.get("imu");
        //odsSensor = hardwareMap.get(OpticalDistanceSensor.class, "sensor_ods");
        //


        //claw = hardwareMap.servo.get("CLAW");

        //relicIO = hardwareMap.crservo.get("RIO");
        //relicUD = hardwareMap.crservo.get("RUD");

        rightSideFront.setDirection(DcMotor.Direction.REVERSE);
        rightSideBack.setDirection(DcMotor.Direction.FORWARD);
        leftSideFront.setDirection(DcMotor.Direction.FORWARD);
        leftSideBack.setDirection(DcMotor.Direction.REVERSE);
        strafe.setDirection(DcMotor.Direction.REVERSE);
        glyphlift.setDirection(DcMotor.Direction.FORWARD);
        glyphleft.setDirection(DcMotor.Direction.FORWARD);
        glyphright.setDirection(DcMotor.Direction.FORWARD);

        leftSideBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSideBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSideFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSideFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafe.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        glyphright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        glyphleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        glyphlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        jewel.setPosition(0);
        glyphdump.setPosition(1);
        glyleft.setPosition(glyleftUp);
        glyright.setPosition(glyrightUp);


    }
}

