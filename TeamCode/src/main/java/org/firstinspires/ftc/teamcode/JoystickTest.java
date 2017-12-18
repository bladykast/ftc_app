

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Joystick Test", group = "TeleOp")
@Disabled

public class JoystickTest extends OpMode {


	public JoystickTest() {

	}

	@Override
	public void init() {


	}

	public void loop()
	{
		float y1 = -gamepad1.left_stick_y;
		float y2 = -gamepad1.right_stick_y;
		float y3 = -gamepad2.left_stick_y;
		float y4 = -gamepad2.right_stick_y;

		y1 = Range.clip(y1, -1, 1);
		y2 = Range.clip(y2, -1, 1);
		y3 = Range.clip(y3, -1, 1);
		y4 = Range.clip(y4, -1, 1);


		telemetry.addData("Text", "*** Robot Data***");
		telemetry.addData("Y1", "Y1:  " + String.format("%.2f", y1));
        telemetry.addData("Y2", "Y2:  " + String.format("%.2f", y2));
	}



	@Override
	public void stop() {

	}

}