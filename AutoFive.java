package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Apollo: Auto 5 ", group="Apollo Autonomous")
public class AutoFive extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        setGameParameters(GamePositions.CRATER,GamePositions.OURCRATER, GamePositions.PARKFORWADRS);
        climbDown();
        moveGoldMineralCraterWithout();
        blockBetween();
        parkForBlockingRobot();

        endAuto();
    }
}
