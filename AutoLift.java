package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Apollo: Auto lift", group="Apollo Autonomous")
public class AutoLift extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();
        RunThread(during, ThreadActions.LIFTUP);
        waitSeconds(9999);

        //waitSeconds(2);

    }

}
