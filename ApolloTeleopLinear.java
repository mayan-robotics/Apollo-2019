package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

/**
 * Apollo Teleop driving.
 */

@TeleOp(name="Teleop Apollo Terror", group="terror")

public class ApolloTeleopLinear extends LinearOpMode{

    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    private MineralVision vision;

    private ElapsedTime runtime = new ElapsedTime();

    static double speedFactor = 1;  // Speed factor
    static final double joyStickLimitPoints = 0.3;  // for better control

    static double normalOrReversDrive = 1;  //

    static final double senderOpenLimitPoint = 7000 ; // Limit so the sender motors wont open to much, by encoder ticks.
    static final double senderCloseLimitPoint = 0 ; // Limit so the sender motors wont open to much, by encoder ticks.
    static final double convertJoystickToPosition = 10 ; //
    int mineralSenderPosition ; // Encoder positions
    int mineralSenderWantedPosition  = 0 ; // Encoder positions

    boolean blockMineral = false;


    @Override
    public void runOpMode() {
        //Hardware init
        robot.init(hardwareMap);
        //robot.setMineralSendMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Vision
        vision = new MineralVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        vision.enable();
        vision.setShowCountours(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Version", robot.Version);
        telemetry.addData("Apollo", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){



            //Controllers drive sticks inputs
            double LeftStickX = gamepad1.left_stick_x*normalOrReversDrive;
            double LeftStickY = -gamepad1.left_stick_y*normalOrReversDrive;  // The joystick goes negative when pushed forwards, so negate it.
            double RightStickX = gamepad1.right_stick_x*normalOrReversDrive;
            double RightStickY = -gamepad1.right_stick_y*normalOrReversDrive;    // The joystick goes negative when pushed forwards, so negate it

            if(gamepad1.left_bumper){
                normalOrReversDrive=-1;
            }else {
                normalOrReversDrive=1;
            }

            if(gamepad1.dpad_up){
                if(speedFactor<1) {
                    speedFactor++;
                }
            }else if(gamepad1.dpad_down){
                if (speedFactor>0){
                    speedFactor--;
                }
            }
            //Drive speed control. Game pad 1, stick buttons.
            if (gamepad1.right_stick_button || gamepad1.left_stick_button){
                speedFactor = 0.5;  // Decrease drive speed
            }
            else{
                speedFactor = 1;    // Normal drive speed
            }

            // Drive modes control. Game pad 1, sticks.
            if (LeftStickX < -joyStickLimitPoints && (Math.abs(LeftStickY) > joyStickLimitPoints) &&
                    (RightStickX < -joyStickLimitPoints && Math.abs(RightStickY) > joyStickLimitPoints))
            {   // If both joysticks are pushed to a left conner. drive diagonal left.
                robot.setDriveMotorsPower(LeftStickY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_LEFT);
                telemetry.addData("Drive", "DIAGONAL_LEFT");
            }
            else if (LeftStickX > joyStickLimitPoints && (Math.abs(LeftStickY) > joyStickLimitPoints) &&
                    (RightStickX > joyStickLimitPoints && Math.abs(RightStickY) > joyStickLimitPoints))
            {   // If both joysticks are pushed to a right conner. drive diagonal right.
                robot.setDriveMotorsPower(LeftStickY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_RIGHT);
                telemetry.addData("Drive", "DIAGONAL_RIGHT");
            }
            else if ((Math.abs(LeftStickX) > Math.abs(LeftStickY) && Math.abs(LeftStickX) > joyStickLimitPoints && Math.abs(LeftStickY)< 0.8) &&
                    (Math.abs(RightStickX) > Math.abs(RightStickY) && Math.abs(RightStickX)> joyStickLimitPoints && Math.abs(RightStickY)< 0.8 &&
                            (RightStickX<0 && LeftStickX<0 || RightStickX>0 && LeftStickX>0 )))
            {   // If both joysticks are pushed to the side. drive sideways.
                robot.setDriveMotorsPower(LeftStickX*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                telemetry.addData("Drive", "Side ways");
            }
            else
            {   // Drive Normally, tank mode.
                robot.setDriveMotorsPower(LeftStickY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
                robot.setDriveMotorsPower(RightStickY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
                telemetry.addData("Drive", "Normal");
            }
            telemetry.update();


            //Mineral graber control. Game pad 2, triggers.
            if (gamepad2.left_trigger<0 && gamepad2.right_trigger<0){
                telemetry.addData("main graber", "stop");
                //robot.mineralGrab.setPower(0);
                robot.setMineralGrabServos(0);
            }
            if (gamepad2.left_trigger>0){
                telemetry.addData("main graber", "grab out");
                telemetry.addData("power graber", gamepad2.left_trigger);
                //robot.mineralGrab.setPower(gamepad2.left_trigger);
                robot.setMineralGrabServos(0.8);
            }
            else if (gamepad2.right_trigger>0){
                telemetry.addData("main graber", "grab in");
                telemetry.addData("power graber", gamepad2.right_trigger);
                //robot.mineralGrab.setPower(-gamepad2.right_trigger);
                robot.setMineralGrabServos(-0.8);
            }else {
                //robot.mineralGrab.setPower(0);
                robot.setMineralGrabServos(0);
            }

            //Mineral sender control. Game pad 2, right stick.
            //if (Math.abs(gamepad1.right_trigger) > 0) {
            robot.mineralSend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(gamepad1.right_trigger > 0 ) {
                //telemetry.addData("1", "here");
                robot.mineralSend.setPower(gamepad1.right_trigger);
                //robot.mineralBoxServo.setPosition(robot.mineralBoxBlock);
            }
            else if(gamepad1.left_trigger > 0 ){
                //telemetry.addData("2", "here");
                robot.mineralSend.setPower(-gamepad1.left_trigger);
                //robot.mineralBoxServo.setPosition(robot.mineralBoxBlock);
            }
            else {
                robot.mineralSend.setPower(0);

                //}
            }
            //else{
            //robot.setMineralSendPower(0);
            //}

            //telemetry.addData("Sender Encoder right", robot.mineralSend.getCurrentPosition());
            //telemetry.update();

            // Mineral blocker control. Game pad 2 bumper.
            if (gamepad2.left_bumper) {
                if (blockMineral){
                    blockMineral=false; // Set Mode of servo to not block minerals.
                }
                else {
                    blockMineral=true;  //Set Mode of servo to block minerals.

                }
            }
/*
        // Keep setting position of servo by the mode its on.
        if (blockMineral){
            robot.blockMineralServo.setPosition(robot.block);
        }
        else {
            robot.blockMineralServo.setPosition(robot.dontBlock);
        }
*/

            if (gamepad2.left_bumper) {
                robot.blockMineralServo.setPosition(robot.dontBlock);
            }
            else {
                robot.blockMineralServo.setPosition(robot.block);

            }


            // Mineral blocker control. Game pad 2 bumper.
            if (gamepad1.x) {
                robot.mineralBoxServo.setPosition(0.30);
            } else if (gamepad1.y) {
                robot.mineralBoxServo.setPosition(0); }
            else if(gamepad1.a){
                robot.mineralBoxServo.setPosition(0.6);
            }



            //Mineral lift Control. Game pad 2, left stick.
            if(-gamepad2.right_stick_y>0.1 ){
                // If left Stick pushed forwards activate mineral push and lift to push out
                robot.push.setPower(-1);
                //robot.lift.setPower(0.3);
                robot.blockMineralServo.setPosition(robot.block);
            }
            else if(-gamepad2.right_stick_y<-0.1){
                if (robot.touchPusher.getState()) {
                    // If left Stick pushed backwards and touch sensor  is not pressed activate mineral push and lift to push in.
                    telemetry.addData("touch", "not pressed");
                    robot.push.setPower(1);
                    //robot.lift.setPower(-0.3);
                } else {
                    // If touch sensor pressed the __ is closed, so stop the motors.
                    telemetry.addData("touch", "pressed");
                    robot.push.setPower(0);
                    robot.lift.setPower(0);
                }
            }


            if(-gamepad2.left_stick_y<0 ) {
                robot.lift.setPower(gamepad2.left_stick_y*0.5);
            }else if(-gamepad2.left_stick_y>0 ){
                robot.lift.setPower(gamepad2.left_stick_y*0.5);

            }
            else{
                robot.push.setPower(0);
                robot.lift.setPower(0);
                //robot.lift.setTargetPosition(liftencoderPosition);
                //robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }





            if(gamepad2.dpad_up && gamepad2.dpad_down){
                robot.climbMotor.setPower(0);
            }
            else if(gamepad2.dpad_up ){
                robot.climbMotor.setPower(1);
            }
            else if(gamepad2.dpad_down){
                robot.climbMotor.setPower(-1);
            }else
            {
                robot.climbMotor.setPower(0);
            }



            telemetry.addData("Sender Encoder push", robot.push.getCurrentPosition());


            telemetry.update();


        }
        vision.disable();

    }

/*

    //Function divides the minerals to gold and silver with servo by camera.
    public void MineralDivideByVision(){
        if(vision.goldMineralFound()== true){
            //robot.mineralsDivider.setPosition(robot.dividerRight);
        }else{
            //robot.mineralsDivider.setPosition(robot.dividerLeft);
        }
    }

/*
    public void encoderMineralSend(double speed, double Distance) {

        robot.setMineralSendMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.setMineralSenderMotorsPosition(Distance);
            // Turn On RUN_TO_POSITION
            robot.setMineralSendMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.setMineralSendPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.mineralSend.isBusy() )){
            }

            // Stop all motion;
            robot.setMineralSendPower(0);

            // Turn off RUN_TO_POSITION
            robot.setMineralSendMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        */
    }

