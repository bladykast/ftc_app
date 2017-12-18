package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.SeriousHardware.glyleftDown;
import static org.firstinspires.ftc.teamcode.SeriousHardware.glyleftUp;
import static org.firstinspires.ftc.teamcode.SeriousHardware.glyphdumpDown;
import static org.firstinspires.ftc.teamcode.SeriousHardware.glyphdumpUp;
import static org.firstinspires.ftc.teamcode.SeriousHardware.glyrightDown;
import static org.firstinspires.ftc.teamcode.SeriousHardware.glyrightUp;


@TeleOp(name = "3736: TeleOp", group = "TeleOp")

public class TeleOp_Comp extends OpMode {

	SeriousHardware robot  = new SeriousHardware();

    public static double jewelPosition, glyphdumpPosition, glyrightPosition, glyleftPosition;
    public static double MAX_SPEED = 1;
    public static double MIN_SPEED = 0.4;
    public static double MOTOR_SPEED = 1;
    public static boolean slow = true;

	@Override
	public void init() {

        robot.init(hardwareMap);

        robot.jewel.setPosition(1);
        robot.glyphdump.setPosition(1);
        robot.glyright.setPosition(0.45);
        robot.glyleft.setPosition(0.55);

        jewelPosition = 1;
        glyphdumpPosition = 1;
        glyrightPosition = 0;
        glyleftPosition = 1;
    }

	public void loop()
	{

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

		if (gamepad1.a) {
			slow = true;
		}

        if (gamepad1.x) slow = false;

        if (slow) MOTOR_SPEED = MIN_SPEED;
        if (!slow) MOTOR_SPEED = MAX_SPEED;

		//Drivetrain Code

        robot.leftSideFront.setPower(y1 * MOTOR_SPEED);
        robot.leftSideBack.setPower(y1 * MOTOR_SPEED);
        robot.rightSideFront.setPower(y2 * MOTOR_SPEED);
        robot.rightSideBack.setPower(y2 * MOTOR_SPEED);
        robot.glyphlift.setPower(y3);

		if (gamepad1.dpad_left)
		{
            robot.strafe.setPower(-1);
        }
		else if (gamepad1.dpad_right)
		{
            robot.strafe.setPower(1);
        } else robot.strafe.setPower(0);


        //Jewel Code

		//if (gamepad2.b)
        //{
        //    jewelPosition = jewelUp;
        //}

		//if (gamepad2.y)
		//{
		//	jewelPosition = jewelDown;
		//}


        //Glyph Code
        //Gamepad 2
        if (gamepad2.x)
		{
			glyphdumpPosition = glyphdumpUp;
		}
		if (gamepad2.a)
		{
			glyphdumpPosition = glyphdumpDown;
		}

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

        if (gamepad1.y) {
            glyrightPosition = glyrightUp;
            glyleftPosition = glyleftUp;
        }
        if (gamepad1.b) {
            glyrightPosition = glyrightDown;
            glyleftPosition = glyleftDown;
        }




		//Relic Code

        //relicUD.setPower(y4);

		//if (gamepad2.dpad_left)
        //{
        //    relicIO.setPower(1);
        //}
        //else if (gamepad2.dpad_right)
        //{
        //    relicIO.setPower(-1);
        //}
        //else relicIO.setPower(0);

        //if (gamepad2.b)
        //{
        //    clawPosition = clawIn;
        //}
        //if (gamepad2.y)
        //{
        //    clawPosition = clawOut;
        //}


        //Super Mario linear code (hold more to go further...)

        //if (gamepad2.x)
        //{
        //    glyphdumpPosition =+ glyphDelta;
        //}

        //if (gamepad2.a)
        //{
        //    glyphdumpPosition =- glyphDelta;
        //}

		//jewelPosition = Range.clip(jewelPosition, JEWEL_MIN_RANGE, JEWEL_MAX_RANGE);
        //glyphdumpPosition = Range.clip(glyphdumpPosition, GLYPHDUMP_MIN_RANGE, GLYPHDUMP_MAX_RANGE);


		//jewel.setPosition(jewelPosition);
        //claw.setPosition(clawPosition);
        robot.glyleft.setPosition(glyleftPosition);
        robot.glyright.setPosition(glyrightPosition);
        robot.glyphdump.setPosition(glyphdumpPosition);

		telemetry.addData("Text", "*** Robot Data***");
		//telemetry.addData("jewel", "jewel:  " + String.format("%.2f", jewelPosition));
        //telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
        telemetry.addData("glyph", "glyph:  " + String.format("%.2f", glyphdumpPosition));
        telemetry.addData("glyleft", "glyph:  " + String.format("%.2f", glyleftPosition));
        telemetry.addData("glyright", "glyph:  " + String.format("%.2f", glyrightPosition));
		telemetry.addData("Motor Speed", "Speed:  " + String.format("%.2f", MOTOR_SPEED));
	}



	@Override
	public void stop() {

	}

}