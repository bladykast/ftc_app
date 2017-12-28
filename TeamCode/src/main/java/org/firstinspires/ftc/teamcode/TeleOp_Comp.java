package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.SeriousHardware.glyStart;
import static org.firstinspires.ftc.teamcode.SeriousHardware.glyphdumpDown;
import static org.firstinspires.ftc.teamcode.SeriousHardware.glyphdumpUp;


@TeleOp(name = "3736: TeleOp", group = "TeleOp")

public class TeleOp_Comp extends OpMode {

    public static double jewelPosition, glyphdumpPosition, glyPosition;
    public static double MAX_SPEED = 1;
    public static double MIN_SPEED = 0.4;
    public static double MOTOR_SPEED = 1;
    public static boolean slow = true;
    SeriousHardware robot = new SeriousHardware();

	@Override
	public void init() {

        robot.init(hardwareMap);

        robot.leftSideBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightSideBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightSideFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftSideFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.strafe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.jewel.setPosition(0);
        robot.glyphdump.setPosition(1);
        robot.gly.setPosition(glyStart);

        jewelPosition = 1;
        glyphdumpPosition = 1;
        glyPosition = 1;
    }

	public void loop()
	{
        /*
	    *  Note from 2017: Yes, forward on the stick is f$#@ing -1. We have had all of our robots
        *  driving backwards for the last 3 years.
        *
        *  Lovely, huh?
        */

		float y1 = -gamepad1.left_stick_y;
		float y2 = -gamepad1.right_stick_y;
		float y3 = -gamepad2.left_stick_y;
		float y4 = -gamepad2.right_stick_y;
		boolean b1 = gamepad2.left_bumper;
		boolean b2 = gamepad2.right_bumper;
		boolean dleft = gamepad1.dpad_left;
		boolean dright = gamepad1.dpad_right;

		y1 = Range.clip(y1, -1, 1);
		y2 = Range.clip(y2, -1, 1);
		y3 = Range.clip(y3, -1, 1);
		y4 = Range.clip(y4, -1, 1);

		//High-Low Speed Code

        if (gamepad1.a) slow = true;
        if (gamepad1.x) slow = false;

        if (slow) MOTOR_SPEED = MIN_SPEED;
        if (!slow) MOTOR_SPEED = MAX_SPEED;

		//Drivetrain Code

        robot.leftSideFront.setPower(y1 * MOTOR_SPEED);
        robot.leftSideBack.setPower(y1 * MOTOR_SPEED);
        robot.rightSideFront.setPower(y2 * MOTOR_SPEED);
        robot.rightSideBack.setPower(y2 * MOTOR_SPEED);
        robot.glyphlift.setPower(y3);

        if (gamepad1.dpad_left) robot.strafe.setPower(-1);
        else if (gamepad1.dpad_right) robot.strafe.setPower(1);
        else robot.strafe.setPower(0);


        //Glyph Code

        //Gamepad 2
        if (gamepad2.x) glyphdumpPosition = glyphdumpUp;
        if (gamepad2.a) glyphdumpPosition = glyphdumpDown;


        //Gamepad 1
        if (gamepad1.left_bumper) {
            robot.glyphleft.setPower(-1);
            robot.glyphright.setPower(-1);
        } else if (gamepad1.right_bumper) {
            robot.glyphleft.setPower(1);
            robot.glyphright.setPower(1);
        } else {
            robot.glyphleft.setPower(0);
            robot.glyphright.setPower(0);
        }


        glyPosition = Range.scale(gamepad1.right_trigger, 0, 1, 1, 0.55);


		//jewel.setPosition(jewelPosition);
        //claw.setPosition(clawPosition);

        robot.gly.setPosition(glyPosition);
        robot.glyphdump.setPosition(glyphdumpPosition);

		telemetry.addData("Text", "*** Robot Data***");
		//telemetry.addData("jewel", "jewel:  " + String.format("%.2f", jewelPosition));
        //telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
        telemetry.addData("Dump", "Pos:  " + String.format("%.2f", glyphdumpPosition));
        telemetry.addData("Wheel", "Pos:  " + String.format("%.2f", glyPosition));
        telemetry.addData("Motor Speed", "Speed:  " + String.format("%.2f", MOTOR_SPEED));
    }



	@Override
	public void stop() {

	}

}