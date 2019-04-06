package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Apollo: Auto Extra", group="Apollo")
public class AutoTestEXTRA extends AutoMain {
    Thread  during = new during();
    Thread  duringTwo = new duringTwo();


    public enum threadActions{
        TOGOLDMINERAL,
        LIFTUP,
        DRIVEFORWARD,
        LIFTDOWN,
        GRABMINERAL,
        PUSH,
        PUTMINERALDRIVE
    }

    public enum threadActionsTwo{
        LIFTDOWN
    }

    double  PUSHSPEED;

    GoldPosition startGoldMineralPosition;


    threadActions currentActionThread;
    threadActions currentActionThreadTwo;

    @Override//mjywgr,2yurd
    public void runOpMode() throws InterruptedException {
        apolloInit();

        waitForStart();

        //currentActionThread=threadActions.GRABMINERAL;
        robot.blockMineralServo.setPosition(robot.block);   // Set Mode of servo to not block minerals.


        PUSHSPEED=1;
        currentActionThread=threadActions.GRABMINERAL;
        during.start();
        waitSeconds(1);
        during.interrupt();
        robot.push.setPower(0);


        //waitSeconds(1);
        telemetry.addData("Yarboa","here");
        telemetry.update();


        liftUntilStuck(-1);
        robot.blockMineralServo.setPosition(robot.dontBlock);   // Set Mode of servo to not block minerals.
        robot.mineralGrab.setPosition(FORWARD);
        waitSeconds(0.4);

        robot.mineralGrab.setPosition(STOP);
        currentActionThread=threadActions.PUTMINERALDRIVE;
        during.start();
        encoderMineralSend(1, 1200);
        robot.mineralSend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("HERE","HERE");
        telemetry.update();
        //robot.mineralGrab.setPosition(STOP);
        robot.mineralBoxServo.setPosition(robot.mineralBoxServoClose);
        telemetry.addData("Yrboa","HERE");
        telemetry.update();

        waitSeconds(5);




        //waitSeconds(10);
        //during.interrupt();


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
                        if(currentActionThread == threadActions.GRABMINERAL){
                            PUSHSPEED=-1;
                            currentActionThread=threadActions.PUSH;
                            duringTwo.start();
                            liftUntilStuck(1);
                            encoderLift(1, (robot.lift.getCurrentPosition() - 100));

                            robot.blockMineralServo.setPosition(robot.block);   // Set Mode of servo to not block minerals.
                            robot.mineralGrab.setPosition(FORWARD);
                            telemetry.addData("Finished1", "here");
                            telemetry.update();
                        }
                        if(currentActionThread == threadActions.PUTMINERALDRIVE){
                            waitSeconds(0.5);
                            gyroDrive(0.6,-20,angelForGyro(0));
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
                    if(currentActionThread==threadActions.PUSH){
                        waitSeconds(0.2);
                        robot.push.setPower(PUSHSPEED);
                        waitSeconds(0.4);
                        robot.push.setPower(-PUSHSPEED);
                        waitSeconds(0.1);
                        robot.push.setPower(PUSHSPEED);
                        robot.mineralGrab.setPosition(FORWARD);
                        waitSeconds(1.5);
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
