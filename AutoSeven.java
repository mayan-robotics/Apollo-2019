package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Apollo: Auto 7 ", group="Apollo Autonomous")
public class AutoSeven extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();
        
        climbDown();
        moveGoldMineralCraterWithout();

        endAuto();
    }
}
