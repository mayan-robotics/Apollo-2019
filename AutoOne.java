package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Apollo: Auto 1 ", group="Apollo Autonomous")
public class AutoOne extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        setGameParameters(GamePositions.DEPOT, GamePositions.OURCRATER, GamePositions.PARKFORWADRS);
        climbDown();
        mainMoveGoldMineral();
        mainPutMarker();
        backToCraterFromDepot();

        endAuto();
    }
}
