package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Apollo: Auto 3", group="Apollo Autonomous")
public class AutoThree extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        setGameParameters(GamePositions.CRATER,GamePositions.OURCRATER, GamePositions.PARKBACKWARDS);
        climbDown();
        moveGoldMineralCraterWithout();
        mainPutMarker();
        waitSeconds(1);
        backToCraterFromDepot();

        endAuto();
    }
}
