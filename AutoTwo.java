package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.WriteAbortedException;


@Autonomous(name="Apollo: Auto 2 ", group="Apollo Autonomous")
public class AutoTwo extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        setGameParameters(GamePositions.CRATER,GamePositions.OURCRATER, GamePositions.PARKFORWADRS);

        climbDown();
        //startGoldMineralPosition=GoldPosition.LEFT;
        //turnToGoldMineral();
        //waitSeconds(2);
        mainMoveGoldMineral();

        telemetry.addData("HEHEHHIO","IHDIU");
        telemetry.update();
        //waitSeconds(2);

        getReadyToGrabMinerals();


        //grabMineralsAndPutInLander();
        grab();
        putMineralsInLander();
        betweenMineralGrab();
        grab();
        putMineralsInLander();
        //grabMineralsAndPutInLander();
        betweenMineralGrab();
        liftDown();


        waitSeconds(1);
        //backToCraterFromDepot();
        //waitSeconds(2);

    }

}
