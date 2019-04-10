package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Apollo: Auto 1", group="Apollo Autonomous")
public class AutoOne extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        setGameParameters(GamePositions.CRATER,GamePositions.OURCRATER, GamePositions.PARKFORWADRS);


        climbDown();

        //startGoldMineralPosition=GoldPosition.MIDDLE;
        //turnToGoldMineral();
        //waitSeconds(2);
        mainMoveGoldMineral();



        //waitSeconds(2);
        mainPutMarker();


        waitSeconds(1);
        backToCraterFromDepot();
        //waitSeconds(2);

    }

}
