package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Apollo: Auto Crater", group="Apollo")
public class AutoDepot extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();
        telemetry.addData("Apollo", "Ready");
        telemetry.update();
        waitForStart();

        apolloRun(false);

    }
}