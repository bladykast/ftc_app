package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;


/**
 * Created by 7t.qr on 12/13/2017.
 */

@Autonomous(name="Red", group ="Autonomous")
public class ColorSensor_RedStat extends LinearOpMode {

    SeriousHardware robot = new SeriousHardware();

    DcMotor rightSideFront, rightSideBack, leftSideFront, leftSideBack, strafe, glyphlift, glyphright, glyphleft;
    Servo glyphdump, jewel, glyright, glyleft;
    DeviceInterfaceModule cdim;
    AdafruitI2cColorSensor sensorRGB;

    boolean Red = true;
    boolean Blue = false;

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

        sensorRGB = (AdafruitI2cColorSensor)hardwareMap.get("sensor_color");

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

        sensorRGB.initialize();

        for(int i = 0; i < 20; ++i) {
            telemetry.addData("Clear", (sensorRGB.alpha()) / 256);
            telemetry.addData("Red  ", (sensorRGB.red()) / 256);
            telemetry.addData("Green", (sensorRGB.green()) / 256);
            telemetry.addData("Blue ", (sensorRGB.blue()) / 256);

            telemetry.update();
        }

        if (sensorRGB.red() > sensorRGB.blue()) {
            rightSideBack.setPower(-0.4);
            leftSideBack.setPower(-0.4);
            rightSideFront.setPower(-0.4);
            leftSideBack.setPower(-0.4);
        } else {
            rightSideBack.setPower(0.4);
            leftSideBack.setPower(0.4);
            rightSideFront.setPower(0.4);
            leftSideBack.setPower(0.4);
        }

        sleep(500);

        rightSideBack.setPower(0);
        leftSideBack.setPower(0);
        rightSideFront.setPower(0);
        leftSideBack.setPower(0);

        sleep(15000);
    }
}