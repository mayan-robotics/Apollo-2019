package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Apollo: Auto 2 ", group="Apollo Autonomous")
public class AutoTwo extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        setGameParameters(GamePositions.DEPOT, GamePositions.OTHERCRATER, GamePositions.PARKFORWADRS);
        climbDown();
        mainMoveGoldMineral();
        mainPutMarker();
        backToCraterFromDepot();

        endAuto();
    }
}
