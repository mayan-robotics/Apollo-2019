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
        telemetry.addData("heweqerc", "werfervver");
        telemetry.update();


        //waitSeconds(2);
        mainPutMarker();

        telemetry.addData("finished", "done");
        telemetry.update();

        //waitSeconds(1);
        backToCraterFromDepot();
        //waitSeconds(2);

    during.interrupt();
    duringTwo.interrupt();

    }
}
