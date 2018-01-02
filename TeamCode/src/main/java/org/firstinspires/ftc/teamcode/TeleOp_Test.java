package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.SeriousHardware.glyStart;
import static org.firstinspires.ftc.teamcode.SeriousHardware.glyphdumpDown;
import static org.firstinspires.ftc.teamcode.SeriousHardware.glyphdumpUp;


@TeleOp(name = "3736: TeleOp Rot Test", group = "TeleOp")

public class TeleOp_Test extends OpMode {

    public static double MAX_SPEED = 1;
    public static double MIN_SPEED = 0.4;
    public static double MOTOR_SPEED = 1;
    public static boolean slow = true;
    SeriousHardware robot = new SeriousHardware();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = .667 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


	@Override
	public void init() {

        robot.init(hardwareMap);

        robot.leftSideFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.leftSideBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightSideFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightSideBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.strafe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


    }

	public void loop()
	{

		telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("STR", "Speed:  " + String.format("%7d", robot.strafe.getCurrentPosition()));
        telemetry.update();
    }


	@Override
	public void stop() {

	}

}