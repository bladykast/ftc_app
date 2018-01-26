

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "H-Slide Outreach Bot", group = "TeleOp")
public class TeleOp_H_Outreach extends OpMode {
    
    DcMotor STR, MR, ML;
    
    public TeleOp_H_Outreach() {

    }

    @Override
    public void init() {

        STR = hardwareMap.dcMotor.get("STR");
        MR = hardwareMap.dcMotor.get("MR");
        ML = hardwareMap.dcMotor.get("ML");
        
        MR.setDirection(DcMotor.Direction.REVERSE);
        ML.setDirection(DcMotor.Direction.FORWARD);
        STR.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop()
    {
        float y1 = -gamepad1.left_stick_y;
        float y2 = -gamepad1.right_stick_y;
        boolean dleft = gamepad1.dpad_left;
        boolean dright = gamepad1.dpad_right;


        ML.setPower(y2);
        MR.setPower(y1);

        if (gamepad1.dpad_left)
        {
            STR.setPower(-1);
        }
        else if (gamepad1.dpad_right)
        {
            STR.setPower(1);
        }
        else STR.setPower(0);

        telemetry.addData("Text", "*** Robot Data***");
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