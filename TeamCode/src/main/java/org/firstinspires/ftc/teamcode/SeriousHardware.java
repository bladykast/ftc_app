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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class SeriousHardware
{
    /* Public OpMode members. */
    DcMotor  rightSideFront, rightSideBack, leftSideFront, leftSideBack, strafe, glyphlift, glyphright, glyphleft = null;
    Servo  glyphdump, jewel, glyright, glyleft = null;

    DeviceInterfaceModule cdim = null;
    ColorSensor sensorRGB = null;

    //Servo claw
    //CrServo relicIO, relicUD

    public double jewelPosition, glyphdumpPosition, glyrightPosition, glyleftPosition;

    public double MAX_SPEED = 1;
    public double MIN_SPEED = 0.4;
    public double MOTOR_SPEED = 1;

    public boolean slow = true;

    public double jewelUp = 1;
    public double jewelDown = 0.5;
    public double glyphdumpUp = 0.1;
    public double glyphdumpDown = 1;
    public double glyrightUp = 0.1;
    public double glyrightDown = 0;
    public double glyleftUp = 0.9;
    public double glyleftDown = 1;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public SeriousHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) {
        // Save reference to Hardware map
        hwMap = hardwareMap;


        strafe = hardwareMap.dcMotor.get("STR");
        rightSideFront = hardwareMap.dcMotor.get("MRF");
        rightSideBack = hardwareMap.dcMotor.get("MRB");
        leftSideFront = hardwareMap.dcMotor.get("MLF");
        leftSideBack = hardwareMap.dcMotor.get("MLB");
        glyphlift = hardwareMap.dcMotor.get("LFT");
        glyphleft = hardwareMap.dcMotor.get("GLYL");
        glyphright = hardwareMap.dcMotor.get("GLYR");

        jewel = hardwareMap.servo.get("JWL");
        glyphdump = hardwareMap.servo.get("GLY");
        glyleft = hardwareMap.servo.get("GLYSL");
        glyright = hardwareMap.servo.get("GLYSR");

        sensorRGB = hardwareMap.colorSensor.get("sensor_color");

        //claw = hardwareMap.servo.get("CLAW");

        //relicIO = hardwareMap.crservo.get("RIO");
        //relicUD = hardwareMap.crservo.get("RUD");

        rightSideFront.setDirection(DcMotor.Direction.FORWARD);
        rightSideBack.setDirection(DcMotor.Direction.REVERSE);
        leftSideFront.setDirection(DcMotor.Direction.REVERSE);
        leftSideBack.setDirection(DcMotor.Direction.FORWARD);
        strafe.setDirection(DcMotor.Direction.REVERSE);
        glyphlift.setDirection(DcMotor.Direction.FORWARD);
        glyphleft.setDirection(DcMotor.Direction.FORWARD);
        glyphright.setDirection(DcMotor.Direction.FORWARD);

        leftSideFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSideFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSideFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSideFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSideFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        jewel.setPosition(1);
        glyphdump.setPosition(1);
        glyright.setPosition(0);
        glyleft.setPosition(1);

        jewelPosition = 1;
        glyphdumpPosition = 1;
        //clawPosition = 0.1;
        glyleftPosition = 1;
        glyrightPosition = 0;
    }
 }

