

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptRampMotorSpeed;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "3736: TeleOp", group = "TeleOp")

public class TeleOp_Comp extends OpMode {


	final static double JEWEL_MIN_RANGE  = 0;
	final static double JEWEL_MAX_RANGE  = 1;
	final static double GLYPHDUMP_MIN_RANGE  = 0;
	final static double GLYPHDUMP_MAX_RANGE  = 1;

	double jewelPosition;
	double glyphdumpPosition;
	double clawPosition;
	double glyleftPosition;
	double glyrightPosition;

	double jewelUp = 1;
	double jewelDown = 0.5;
	double glyphdumpUp = 0.1;
    double glyphdumpDown = 1;
    double clawIn;
    double clawOut;

	DcMotor strafe;
	DcMotor rightSideFront;
	DcMotor rightSideBack;
	DcMotor leftSideFront;
	DcMotor leftSideBack;
    DcMotor glyphlift;
	DcMotor glyphleft;
	DcMotor glyphright;

	//CRServo relicIO;
	//CRServo relicUD;

	Servo glyphdump;
    //Servo jewel;
    Servo glyleft;
    Servo glyright;
    //Servo claw;


	public TeleOp_Comp() {

	}

	@Override
	public void init() {

		strafe = hardwareMap.dcMotor.get("STR");
		rightSideFront = hardwareMap.dcMotor.get("MRF");
		rightSideBack = hardwareMap.dcMotor.get("MRB");
		leftSideFront = hardwareMap.dcMotor.get("MLF");
		leftSideBack = hardwareMap.dcMotor.get("MLB");
		glyphlift = hardwareMap.dcMotor.get("LFT");
        glyphleft = hardwareMap.dcMotor.get("GLYL");
        glyphright = hardwareMap.dcMotor.get("GLYR");

        //jewel = hardwareMap.servo.get("JWL");
        glyphdump = hardwareMap.servo.get("GLY");
		glyleft = hardwareMap.servo.get("GLYSL");
		glyright = hardwareMap.servo.get("GLYSR");

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

		//jewelPosition = 1;
		glyphdumpPosition = 1;
		clawPosition = 0.1;


		//NOT LEGAL DO NOT KEEP THIS CODE FOR COMPETITION

		glyleft.setPosition(1);
		glyright.setPosition(0);


	}

	public void loop()
	{

		float y1 = gamepad1.left_stick_y;
		float y2 = gamepad1.right_stick_y;
		float y3 = gamepad2.left_stick_y;
		float y4 = gamepad2.right_stick_y;
		boolean b1 = gamepad2.left_bumper;
		boolean b2 = gamepad2.right_bumper;
		boolean dleft = gamepad1.dpad_left;
		boolean dright = gamepad1.dpad_right;

		y1 = Range.clip(y1, -1, 1);
		y2 = Range.clip(y2, -1, 1);
		y3 = Range.clip(y3, -1, 1);
		y4 = Range.clip(y4, -1, 1);

		y1 = (float)scaleInput(y1);
		y2 = (float)scaleInput(y2);
		y3 = (float)scaleInput(y3);
		y4 = (float)scaleInput(y4);


		//Drivetrain Code

		leftSideFront.setPower(y1);
		leftSideBack.setPower(y1);
		rightSideFront.setPower(y2);
		rightSideBack.setPower(y2);
		glyphlift.setPower(y3);

		if (gamepad1.dpad_left)
		{
			strafe.setPower(-1);
		}
		else if (gamepad1.dpad_right)
		{
			strafe.setPower(1);
		}
		else strafe.setPower(0);


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
        if (gamepad1.left_bumper)
        {
            glyphleft.setPower(-1);
            glyphright.setPower(-1);
        }
        else if (gamepad1.right_bumper)
        {
            glyphleft.setPower(1);
            glyphright.setPower(1);
        }
        else
        {
            glyphleft.setPower(0);
            glyphright.setPower(0);
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
        glyphdump.setPosition(glyphdumpPosition);

		telemetry.addData("Text", "*** Robot Data***");
		//telemetry.addData("jewel", "jewel:  " + String.format("%.2f", jewelPosition));
        //telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
		telemetry.addData("glyph", "glyph:  " + String.format("%.2f", glyphdumpPosition));
	}



	@Override
	public void stop() {

	}

	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		int index = (int) (dVal * 16.0);

		if (index < 0) {
			index = -index;
		}

		if (index > 16) {
			index = 16;
		}

		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		return dScale;
	}
}