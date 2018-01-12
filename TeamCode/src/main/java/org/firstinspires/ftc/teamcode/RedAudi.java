package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Created by 7t.qr on 12/13/2017.
 */

@Autonomous(name = "Red Audience Side", group = "Autonomous")
public class RedAudi extends LinearOpMode {

    SeriousHardware robot = new SeriousHardware();

    boolean Red = true;
    boolean Blue = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.jewel.setPosition(0);
        robot.glyphdump.setPosition(1);
        robot.gly.setPosition(0.55);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        sleep(2000);

        robot.jewel.setPosition(0.65);

        sleep(2000);

        for(int i = 0; i < 20; ++i) {
            telemetry.addData("Clear", (robot.sensorRGB.alpha()) / 256);
            telemetry.addData("Red  ", (robot.sensorRGB.red()) / 256);
            telemetry.addData("Green", (robot.sensorRGB.green()) / 256);
            telemetry.addData("Blue ", (robot.sensorRGB.blue()) / 256);

            telemetry.update();
        }

        if (robot.sensorRGB.red() >= robot.sensorRGB.blue()) {
            GoForward(0.4);
        } else {
            GoBackward(0.4);
        }

        sleep(500);

        robot.jewel.setPosition(0);

        Stop();

        sleep(15000);
    }

    public void GoForward(double power) {
        robot.rightSideBack.setPower(power);
        robot.leftSideBack.setPower(power);
        robot.rightSideFront.setPower(power);
        robot.leftSideFront.setPower(power);
    }

    public void GoBackward(double power) {
        GoForward(-power);
    }

    public void Stop() {
        robot.rightSideBack.setPower(0);
        robot.leftSideBack.setPower(0);
        robot.rightSideFront.setPower(0);
        robot.leftSideFront.setPower(0);
        robot.strafe.setPower(0);
    }

    public void Strafe(double power) {
        robot.strafe.setPower(power);
    }
}