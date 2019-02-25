package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Apollo: Auto Depot Our Crater", group="Apollo")
public class AutoDepotOurCrater extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();
        TURNRIGHTORLEFT = 1;    // Turn right for the other crater.

        waitForStart();

        apolloRun(false);   // Run auto of Depot.

    }
}