package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Apollo: Auto Depot Our Crater", group="Apollo")
public class AutoDepotOurCrater extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();
        //telemetry.addData("Apollo", "Ready");
        //telemetry.update();
        TURNRIGHTORLEFT = 1;
        waitForStart();

        apolloRun(false);

    }
}