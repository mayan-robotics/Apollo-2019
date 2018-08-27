package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Check robot hardware.
 */

@Autonomous(name="Hardware check", group="Apollo")
public class HardwareCheck extends AutoMain{
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Apollo", "Ready");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

    }

}