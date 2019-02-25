package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Apollo: Auto Depot Other Crater", group="Apollo")
public class AutoDepotOtherCrater extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();
        TURNRIGHTORLEFT = -1;   // Turn left for the other crater.

        waitForStart();

        apolloRun(false);   // Run auto of Depot.
    }
}