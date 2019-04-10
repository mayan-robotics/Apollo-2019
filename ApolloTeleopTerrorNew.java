package org.firstinspires.ftc.teamcode;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Apollo Teleop driving.
 */

@TeleOp(name="Teleop New Terror", group="terror")

public class ApolloTeleopTerrorNew extends RobotFunctions {


    static double speedFactor = 1;  // Speed factor
    static double normalOrReversDrive = 1;  // Drive normal or revers

    static final double joyStickLimitPoints = 0.3;  // For better control

    volatile boolean climbMotorInUse = false;
    volatile boolean climbThreadActive = false;

    volatile boolean moveMineralsThreadActive = false;
    volatile boolean moveMineralsInUse = false;

    volatile boolean liftDownThreadActive = false;
    volatile boolean liftDownInUse = false;
    volatile boolean buttonClicked = true;
    volatile boolean threadOn = true;

    volatile boolean OutThread = true;
    int PUSHSPEED = 1;

    boolean sevoblock;
    double pushPower;



    @Override
    public void runOpMode() {
        Thread  climb = new climb();                    // Climb Thread
        Thread  moveMinerals = new moveMinerals();      // Move minerals Thread
        ElapsedTime runtime = new ElapsedTime();

        //Hardware init
        robot.init(hardwareMap);

        //robot.setNotDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.mineralSend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.push.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        TelemetryRobotStartStatus();

        /** Wait For Start **/
        waitForStart();

        robot.InitServoes();        // Set all servos positions

        while (opModeIsActive())
        {
            if(gamepad2.x){
                if(threadOn == true){
                    threadOn= false;
                }else{
                    threadOn= true;
                }
            }

            DrivingFeatures();
            MainDriving();

            GraberControl();
            ExtrusionsControl();
            pushExtrusionsControl(runtime);
            MineralBoxControl();
            ServoBlockMineralsControl();
            LiftControl();
            GeneralRobotActions();


            //ResetEncodersButton();

            ClimbThreadActivate(climb);
            ClimbSystemControl(climb);
            MoveMineralsMagicButtonControl(moveMinerals);

            //mineralPassThread(moveMinerals);


            TelemetryRobotStatus();
        }

        moveMinerals.interrupt();
        climb.interrupt();


    }






    public void MainDriving(){
        double LeftStickX;  //= gamepad1.left_stick_x * normalOrReversDrive;
        double LeftStickY;  //= -gamepad1.left_stick_y * normalOrReversDrive;  // The joystick goes negative when pushed forwards, so negate it.
        double RightStickX; //= gamepad1.right_stick_x * normalOrReversDrive;
        double RightStickY; //= -gamepad1.right_stick_y * normalOrReversDrive;    // The joystick goes negative when pushed forwards, so negate it
        //Controllers drive sticks inputs
        if(normalOrReversDrive== -1) {
            RightStickX = gamepad1.left_stick_x * normalOrReversDrive;
            RightStickY = -gamepad1.left_stick_y * normalOrReversDrive;  // The joystick goes negative when pushed forwards, so negate it.
            LeftStickX = gamepad1.right_stick_x * normalOrReversDrive;
            LeftStickY = -gamepad1.right_stick_y * normalOrReversDrive;    // The joystick goes negative when pushed forwards, so negate it
        }else{
            LeftStickX = gamepad1.left_stick_x * normalOrReversDrive;
            LeftStickY = -gamepad1.left_stick_y * normalOrReversDrive;  // The joystick goes negative when pushed forwards, so negate it.
            RightStickX = gamepad1.right_stick_x * normalOrReversDrive;
            RightStickY = -gamepad1.right_stick_y * normalOrReversDrive;    // The joystick goes negative when pushed forwards, so negate it
        }
        /** Main Driving **/
        // Drive modes control. Game pad 1, sticks.
        if ((LeftStickX < -joyStickLimitPoints) && ((Math.abs(LeftStickY) > joyStickLimitPoints)) &&
                ((RightStickX < -joyStickLimitPoints) && (Math.abs(RightStickY) > joyStickLimitPoints))) {       // If both joysticks are pushed to a left conner. drive diagonal left.
            robot.setDriveMotorsPower(LeftStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_LEFT);
            telemetry.addData("Drive", "DIAGONAL_LEFT");
        }/* else if (LeftStickX > joyStickLimitPoints && (Math.abs(LeftStickY) > joyStickLimitPoints) &&
                (RightStickX > joyStickLimitPoints && Math.abs(RightStickY) > joyStickLimitPoints)) {       // If both joysticks are pushed to a right conner. drive diagonal right.
            robot.setDriveMotorsPower(LeftStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_RIGHT);
            telemetry.addData("Drive", "DIAGONAL_RIGHT");
        }*/ else if ((Math.abs(LeftStickX) > Math.abs(LeftStickY) && Math.abs(LeftStickX) > joyStickLimitPoints && Math.abs(LeftStickY) < 0.8) &&
                (Math.abs(RightStickX) > Math.abs(RightStickY) && Math.abs(RightStickX) > joyStickLimitPoints && Math.abs(RightStickY) < 0.8 &&
                        ((RightStickX < 0 && LeftStickX < 0) || (RightStickX > 0 && LeftStickX > 0)))) {       // If both joysticks are pushed to the side. drive sideways.
            robot.setDriveMotorsPower(LeftStickX * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            telemetry.addData("Drive", "Side ways");
        } else {   // Drive Normally, tank mode.
            robot.setDriveMotorsPower(LeftStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
            robot.setDriveMotorsPower(RightStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
            //telemetry.addData("Drive", "Normal");
        }
    }

    public void DrivingFeatures(){
        // Game pad 1, left bumper. Revers all driving control.
        if (gamepad1.left_bumper) {
            normalOrReversDrive = -1;
        } else {
            normalOrReversDrive = 1;
        }

        //Drive speed control. Game pad 1, stick buttons.
        if (gamepad1.right_stick_button || gamepad1.left_stick_button) {
            speedFactor = 0.5;  // Decrease drive speed for better accuracy.
        } else {
            speedFactor = 1;    // Normal drive speed.
        }
    }

    public void GraberControl(){
        //Mineral graber control. Game pad 2, triggers.
        if (gamepad2.left_trigger < 0 && gamepad2.right_trigger < 0) {
            robot.mineralGrab.setPosition(FORWARD);
        }else if (gamepad2.right_trigger > 0) {
            robot.mineralGrab.setPosition(FORWARD);
        } else if (gamepad2.left_trigger > 0) {
            robot.mineralGrab.setPosition(BACKWARDS);
        } else if(!moveMineralsInUse){
            robot.mineralGrab.setPosition(STOP);
        }
    }

    public void ExtrusionsControl(){
        // Game Pad 1, triggers. Extrusions control.
        if (gamepad1.right_trigger > 0.1 )
        {   // Right trigger pushed, open extrusions.
            robot.mineralSend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.mineralSend.setPower(gamepad1.right_trigger);
            if(robot.mineralSend.getCurrentPosition()<450 && !gamepad1.y){
                robot.mineralBoxServo.setPosition(1);
            }
        } else if (gamepad1.left_trigger > 0.1)
        {
            robot.mineralSend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Left trigger pushed, close extrusions.
            robot.mineralSend.setPower(-0.7);
            //robot.mineralBoxServo.setPosition(robot.mineralBoxServoOpen);
        } else {
            robot.mineralSend.setPower(0);
            robot.mineralSend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        // Yotam helped
        }
    }


    public void ExtrusionsControlWithoutEnoders(){
        // Game Pad 1, triggers. Extrusions control.
        if (gamepad1.right_trigger > 0.1 )
        {   // Right trigger pushed, open extrusions.
            robot.mineralSend.setPower(gamepad1.right_trigger);

        } else if (gamepad1.left_trigger > 0.1)
        {
            // Left trigger pushed, close extrusions.
            robot.mineralSend.setPower(-1);
            //robot.mineralBoxServo.setPosition(robot.mineralBoxServoOpen);
        } else {
            robot.mineralSend.setPower(0);
            robot.mineralSend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        // Yotam helped
        }
    }

    public void pushExtrusionsControl(ElapsedTime runtime) {

        if(pushPower<1) {
            pushPower = (runtime.seconds()/2);
        }else{
            pushPower=1;
            }
        if (!moveMineralsInUse){
            if (-gamepad2.right_stick_y < -0.2 && !buttonClicked) {

                robot.push.setPower(0.7);
                //robot.push.setPower(gamepad2.right_stick_y* 0.6);
                //robot.blockMineralServo.setPosition(robot.block);
            } else if (-gamepad2.right_stick_y > 0.2 ) {
                runtime.reset();
                //telemetry.addData("YARVOA","YARBOA2");
                //telemetry.update();
                buttonClicked = false;
                robot.push.setPower(gamepad2.right_stick_y);
            } else {
                runtime.reset();
                if (!moveMineralsInUse) {
                    robot.push.setPower(0);
                    robot.push.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
        }
    }

    public void MineralBoxControl(){
        // Game pad 1, buttons. Mineral box control.
        if (gamepad1.y) {
            robot.mineralBoxServo.setPosition(robot.mineralBoxServoClose);
        } else if (gamepad1.a) {
            robot.mineralBoxServo.setPosition(robot.mineralBoxServoOpen);
        }else if(gamepad1.x){
            robot.mineralBoxServo.setPosition(0.9);
        }else if(gamepad1.b){

        }
    }

    public void ServoBlockMineralsControl(){
        // Mineral blocker control. Game pad 2 bumper.
        if (gamepad2.left_bumper) {
            robot.blockMineralServo.setPosition(robot.dontBlock);   // Set Mode of servo to not block minerals.
        } else{
            robot.blockMineralServo.setPosition(robot.block);   //Set Mode of servo to block minerals.
        }
    }

    public void ResetEncodersButton(){
        // Reset encoders ,game pad 1 b.
        if(gamepad1.b){
            try {
                RestartAllEncoders();
            }catch (InterruptedException e) { }
        }

    }


    public void LiftControl(){
        //telemetry.addData("time",runtime.seconds());
        //telemetry.update();
        // Mineral lift Control. Game pad 2, left stick.
        if (-gamepad2.left_stick_y < -0.4 ) {
            robot.lift.setPower(-gamepad2.left_stick_y );
        } else if (-gamepad2.left_stick_y > 0.4  ){
            robot.lift.setPower(-gamepad2.left_stick_y );
        }else if(gamepad2.a) {
            robot.lift.setPower(1);
        }else if(gamepad2.b){
            robot.lift.setPower(-1);
        } else{
            if(!moveMineralsInUse) {
                robot.lift.setPower(0);
                //robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        // Yotam helped
            }
        }
    }

    public void ClimbSystemControl(Thread climb){
        // Climb Control. Game pad 2, up down buttons.
        if (gamepad2.dpad_up && gamepad2.dpad_down) {
            if (!climbMotorInUse) {
                robot.climbMotor.setPower(0);
            }
        } else if (gamepad2.dpad_down) {
            climb.interrupt();
            climbThreadActive = false;
            climbMotorInUse = false;
            robot.setAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.climbMotor.setPower(-1);
        } else if (gamepad2.dpad_up) {
            climb.interrupt();
            climbThreadActive = false;
            climbMotorInUse = false;
            robot.setAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.climbMotor.setPower(1);
        } else {
            if (!climbMotorInUse) {
                robot.climbMotor.setPower(0);
                robot.climbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

        }
    }

    public void ClimbThreadActivate(Thread climb){
        // Thread auto climb set
        if((!climbThreadActive) && (!gamepad2.dpad_up) && (!gamepad2.dpad_down)){
            //climb.start();
        }
    }


    public void MoveMineralsMagicButtonControl(Thread moveMinerals){
        if(gamepad2.right_bumper){
            if (!moveMineralsThreadActive) {
                moveMinerals.start();
            }
        }else if(gamepad2.y){
            if (!moveMineralsThreadActive) {
                moveMinerals.start();
            }
        }else {
            if(moveMineralsThreadActive) {
                moveMineralsThreadActive=false;
                moveMineralsInUse=false;
                moveMinerals.interrupt();
            }
        }

    }


    public void mineralPassThread(Thread moveMinerals){
        if(threadOn){
            if(!moveMineralsThreadActive) {
                moveMinerals.start();
            }
        }else {
            if(moveMineralsThreadActive){
                moveMinerals.interrupt();
                moveMineralsInUse=false;
            }
        }

        if(gamepad2.right_bumper){
            if(OutThread) {
                moveMinerals.interrupt();
                moveMineralsInUse=false;
            }
        }
    }

    public void GeneralRobotActions(){
        // When the Extrusions are dow set their position.
        if((-180<robot.mineralSend.getCurrentPosition()) && (robot.mineralSend.getCurrentPosition()< 180)){
            robot.mineralBoxServo.setPosition(robot.mineralBoxServoOpen);
        }

        // Get the button state.
        if(!robot.touchPusher.getState()){
            buttonClicked=true;
        }
    }


    public void TelemetryRobotStatus(){
        telemetry.addData("Encoder lift", robot.lift.getCurrentPosition());
        telemetry.addData("Encoder push", robot.push.getCurrentPosition());
        telemetry.addData("Encoder climb", robot.climbMotor.getCurrentPosition());
        telemetry.addData("Encoder Sender", robot.mineralSend.getCurrentPosition());
        telemetry.addData("gyro", GetGyroAngle());
        telemetry.update();
    }


    public void TelementryButtonCheck(){
        if (robot.touchPusher.getState()) {
            // If left Stick pushed backwards and touch sensor  is not pressed activate mineral push and lift to push in.
            telemetry.addData("touch", "not pressed");
        }else {
            telemetry.addData("touch", "pressed");
        }
    }





    /** --------------------------------------------------------------- **/



    private class climb extends Thread
    {
        public climb()
        {
            this.setName("climb");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            try {
                while (opModeIsActive()) {
                    while (!isInterrupted()) {
                        climbThreadActive=true;
                        if (gamepad2.y) {
                            climbMotorInUse=true;
                            robot.climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            encoderClimb(1, robot.climbLanderPosition);
                            climbMotorInUse=false;
                        }else{
                            Thread.sleep(threadSleepTimeMS);
                        }
                    }
                    Thread.sleep(threadSleepTimeMS);
                }
            }catch (InterruptedException e) { }
        }
    }


    private class moveMinerals extends Thread
    {
        public moveMinerals() {
            this.setName("moveMinerals");
        }
        @Override
        public void run()
        {
            try {
                //while (opModeIsActive()) {
                    Thread timepush = new timepush();      // Move minerals Thread
                    moveMineralsThreadActive = true;

                    if (gamepad2.right_bumper) {
                        moveMineralsInUse = true;
                        //mineralUp();
                        //waitSeconds(5);
                        PUSHSPEED=1;
                        timepush.start();
                        liftUntilStuck(-1);
                        robot.blockMineralServo.setPosition(robot.dontBlock);   // Set Mode of servo to not block minerals.
                        robot.mineralGrab.setPosition(FORWARD);
                        waitSeconds(5);

                        telemetry.addData("Finished1", "here");
                        telemetry.update();
                        moveMineralsInUse = false;


                    } else if(gamepad2.y) {
                        OutThread=true;
                        moveMineralsInUse = true;

                        PUSHSPEED=-1;
                        timepush.start();
                        liftUntilStuck(1);
                        encoderLift(1, (robot.lift.getCurrentPosition() - 100));

                        robot.blockMineralServo.setPosition(robot.block);   // Set Mode of servo to not block minerals.
                        robot.mineralGrab.setPosition(FORWARD);
                        telemetry.addData("Finished1", "here");
                        telemetry.update();

                        waitSeconds(100);
                        moveMineralsInUse = false;
                        OutThread=false;
                    //}
                /*
                if(gamepad2.right_bumper) {
                    moveMineralsThreadActive = true;
                    moveMineralsInUse = true;
                    mineralUp();
                    waitSeconds(5);
                    telemetry.addData("Finished1", "here");
                    telemetry.update();
                    moveMineralsInUse = false;
                }
                else if(gamepad2.dpad_left){
                    liftUntilStuck(1);
                    waitSeconds(3);
                }
                */
                }

            }catch (InterruptedException e){
                telemetry.addData("Interrupt",e);
                telemetry.update();
            }

        }
    }

    public void callThread(Thread timepush){
        timepush.start();
    }


    private class timepush extends Thread
    {
        public timepush() {
            this.setName("timepush");
        }
        @Override
        public void run()
        {
            try {

                telemetry.addData("HEREEE","HEERE");
                telemetry.update();
                waitSeconds(0.2);
                robot.push.setPower(PUSHSPEED);
                //waitSeconds(0.5);
                robot.mineralGrab.setPosition(FORWARD);
                waitSeconds(1.5);

            }catch (InterruptedException e){
                telemetry.addData("Interrupt",e);
                telemetry.update();
            }

        }
    }


/*
    private class liftDown extends Thread
    {
        public liftDown()
        {
            this.setName("liftDown");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            try{
                while (opModeIsActive()) {
                    while (!isInterrupted()) {
                        liftDownThreadActive = true;
                        if(gamepad2.dpad_left){
                            liftDownInUse = true;
                            encoderLift(1,210);
                            liftDownInUse = false;
                        }else {
                            Thread.sleep(thraedSleepTimeMS);
                        }
                        //liftDownInUse = false;
                        Thread.sleep(thraedSleepTimeMS);
                    }
                }

            }catch (InterruptedException e) {
            }

        }
    }
*/

    public float GetGyroAngle(){
        Orientation angles =robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
    }


}

