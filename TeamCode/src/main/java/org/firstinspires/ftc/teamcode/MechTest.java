

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Mecanum Test", group = "TeleOp")
public class MechTest extends OpMode {

    MecHardware robot = new MecHardware();

    public MechTest() {}

    @Override
    public void init(){

        robot.init(hardwareMap);

    }

    public void loop() {
        robot.MLF.setPower(1);
        robot.MRF.setPower(1);
        robot.MLB.setPower(1);
        robot.MRB.setPower(1);
    }


    @Override
    public void stop() {
    }
}