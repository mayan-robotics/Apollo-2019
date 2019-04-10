package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Apollo: Auto 3 ", group="Apollo Autonomous")
public class AutoThree extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();
    while (opModeIsActive()) {
        setGameParameters(GamePositions.DEPOT, GamePositions.OURCRATER, GamePositions.PARKFORWADRS);

        climbDown();
        //startGoldMineralPosition=GoldPosition.LEFT;
        //turnToGoldMineral();
        //waitSeconds(2);
        mainMoveGoldMineral();
        telemetry.addData("heweqerc", "werfervver");
        telemetry.update();


        //waitSeconds(2);
        mainPutMarker();

        telemetry.addData("finished", "done");
        telemetry.update();

        //waitSeconds(1);
        backToCraterFromDepot();
        //waitSeconds(2);
    }
    during.interrupt();
    duringTwo.interrupt();

    }
}
