package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Apollo: Auto 1 ", group="Apollo Autonomous")
public class AutoOne extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
            apolloInit();

            setGameParameters(GamePositions.DEPOT, GamePositions.OURCRATER, GamePositions.PARKFORWADRS);

            climbDown();

            mainMoveGoldMineral();

            mainPutMarker();


            //waitSeconds(1);
            backToCraterFromDepot();

            telemetry.addData("finished", "done");
            telemetry.update();
            //waitSeconds(2);

            during.interrupt();
            duringTwo.interrupt();

    }
}
