package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.WriteAbortedException;


@Autonomous(name="Apollo: Auto 4 ", group="Apollo Autonomous")
public class AutoFour extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        setGameParameters(GamePositions.CRATER,GamePositions.OURCRATER, GamePositions.PARKFORWADRS);

        climbDown();
        //startGoldMineralPosition=GoldPosition.LEFT;
        //turnToGoldMineral();
        //waitSeconds(2);
        //mainMoveGoldMineral();

        telemetry.addData("HEHEHHIO","IHDIU");
        telemetry.update();
        //waitSeconds(2);

        moveGoldMineralCraterWithout();
        getReadyToGrabMinerals();


        //grabMineralsAndPutInLander();
        grab();
        putMineralsInLander();
        betweenMineralGrab();
        liftDown();
        //grabWitoutLift();
        //grab();
        //putMineralsInLander();
        //grabMineralsAndPutInLander();
        //betweenMineralGrab();
        //liftDown();


        //waitSeconds(1);
        //backToCraterFromDepot();
        //waitSeconds(2);

    }

}
