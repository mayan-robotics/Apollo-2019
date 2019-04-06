package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="Apollo: Auto Crater", group="Apollo")
@Disabled
public class AutoCrater extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();
//hi this is roi
        apolloRun(true);    // Run auto of crater.

    }
}
