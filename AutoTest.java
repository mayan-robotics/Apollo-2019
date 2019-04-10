package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;



@Autonomous(name="Apollo: Auto TEST", group="Apollo")
public class AutoTest extends AutoMain {
    Thread  during = new during();
    Thread  duringTwo = new duringTwo();


    public enum threadActions{
        TOGOLDMINERAL,
        LIFTUP,
        DRIVEFORWARD,
        LIFTDOWN
    }

    public enum threadActionsTwo{
        LIFTDOWN
    }

    GoldPosition startGoldMineralPosition;


    threadActions currentActionThread;
    threadActions currentActionThreadTwo;

    @Override
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        setGameParameters(GamePositions.DEPOT,GamePositions.OURCRATER, GamePositions.PARKFORWADRS);

        grab();
        putMineralsInLander();
        betweenMineralGrab();
        grab();
        putMineralsInLander();
        //grabMineralsAndPutInLander();
        betweenMineralGrab();
        liftDown();


        /*
        waitSeconds(999);
        robot.push.setPower(1);
        waitSeconds(2);
        pushClose(-1);
        waitSeconds(999);



        startGoldMineralPosition=GoldPosition.LEFT;
        currentActionThread=threadActions.TOGOLDMINERAL;
        during.start();

        turnAwayFromLender(startGoldMineralPosition);
        //encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, -40);
        //gyroTurn(TURN_SPEED,angelForGyro(90+26));


        robot.push.setPower(1);
        waitSeconds(2);
        robot.push.setPower(0);
        waitSeconds(1);


        //waitSeconds(0.5);

        during.interrupt();


        currentActionThread=threadActions.LIFTUP;
        during.start();

        robot.mineralGrab.setPosition(STOP);

        turnByGyro(TURN_SPEED,angelForGyro(-getTheGyroAngleToTurnToTheGoldMineral(startGoldMineralPosition)));


        gyroDrive(1,45,angelForGyro(0));

        goFromCraterToDepot(-1);

        currentActionThreadTwo=threadActions.LIFTDOWN;
        duringTwo.start();
        //waitSeconds(2);
        currentActionThread=threadActions.DRIVEFORWARD;


        during.start();
        robot.push.setPower(1);
        waitSeconds(2);
        robot.mineralGrab.setPosition(BACKWARDS);

        robot.push.setPower(0);
        waitSeconds(1);




        //waitSeconds(10);
        //during.interrupt();

*/

    }

    private class during extends Thread
    {
        public during()
        {
            this.setName("during");

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            if (opModeIsActive()) {
                //while (!isInterrupted()) {
                    try {
                        if (currentActionThread == threadActions.TOGOLDMINERAL) {
                            waitSeconds(0.3);
                            liftDown();
                            //robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);                               waitSeconds(1);
                            //robot.mineralGrab.setPosition(FORWARD);
                        }
                        if (currentActionThread == threadActions.LIFTUP) {
                            //waitSeconds(0.2);
                            //robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            liftUntilStuck(-1);
                        }
                        if (currentActionThread == threadActions.DRIVEFORWARD) {
                            gyroDrive(DRIVE_SPEED,150,angelForGyro(0));
                            //liftDown();
                        }
                    }catch (InterruptedException e){
                        telemetry.addData("Interrupt",e);
                        telemetry.update();
                    }

                //}
            }

        }
    }


    private class duringTwo extends Thread
    {
        public duringTwo()
        {
            this.setName("duringTwo");

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            if (opModeIsActive()) {
                //while (!isInterrupted()) {
                try {
                    if (currentActionThreadTwo == threadActions.LIFTDOWN) {
                        //waitSeconds(0.5);
                        liftDown();
                        //robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);                               waitSeconds(1);
                        //robot.mineralGrab.setPosition(FORWARD);
                    }
                }catch (InterruptedException e){
                    telemetry.addData("Interrupt",e);
                    telemetry.update();
                }

                //}
            }

        }
    }
}
