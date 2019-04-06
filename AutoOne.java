package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Apollo: Auto One", group="Apollo Autonomous")
public class AutoOne extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        setGameParameters(GamePositions.CRATER,GamePositions.OURCRATER, GamePositions.PARKFORWADRS);

        climbDown();
        startGoldMineralPosition=GoldPosition.LEFT;
        //turnToGoldMineral();
        //waitSeconds(2);
        mainMoveGoldMineral();


        //waitSeconds(2);
        mainPutMarker();

        telemetry.addData("finished","done");
        telemetry.update();

        waitSeconds(1);
        backToCraterFromDepot();
        //waitSeconds(2);

    }

}
