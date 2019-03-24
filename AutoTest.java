package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Apollo: Auto TEST", group="Apollo")
public class AutoTest extends AutoMain {
// yair is white as kir
    Thread  during = new during();

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        liftDown();
        waitSeconds(3);
        passMinrals();
    }

    private class during extends Thread
    {
        public during()
        {
            this.setName("duringClimb");

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            if (opModeIsActive()) {
                while (!isInterrupted()) {

                }
            }

        }
    }
}
