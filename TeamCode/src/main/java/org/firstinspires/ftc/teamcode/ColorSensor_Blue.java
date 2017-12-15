package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by 7t.qr on 12/13/2017.
 */

@Autonomous(name="Blue", group ="Concept")
public class ColorSensor_Blue extends LinearOpMode {

    SeriousHardware robot  = new SeriousHardware();

    DcMotor  rightSideFront, rightSideBack, leftSideFront, leftSideBack, strafe, glyphlift, glyphright, glyphleft;
    Servo  glyphdump, jewel, glyright, glyleft;
    DeviceInterfaceModule cdim;
    ColorSensor sensorRGB;

    boolean Blue = true;
    boolean Red = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

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

        jewel.setPosition(1);
        glyphdump.setPosition(1);
        glyright.setPosition(0.45);
        glyleft.setPosition(0.55);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        sleep(2000);

        jewel.setPosition(0.35);

        sleep(2000);

        for(int i = 0; i < 20; ++i) {
            telemetry.addData("Clear", sensorRGB.alpha());
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());

            telemetry.update();
        }

        if (sensorRGB.red() > 100) {
            rightSideBack.setPower(0.4);
            leftSideBack.setPower(0.4);
            rightSideFront.setPower(0.4);
            leftSideBack.setPower(0.4);
        } else {
            rightSideBack.setPower(-0.4);
            leftSideBack.setPower(-0.4);
            rightSideFront.setPower(-0.4);
            leftSideBack.setPower(-0.4);
        }

        sleep(500);

        rightSideBack.setPower(0);
        leftSideBack.setPower(0);
        rightSideFront.setPower(0);
        leftSideBack.setPower(0);

        sleep(15000);


  }
}

