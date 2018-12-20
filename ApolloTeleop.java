package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

/**
 * Apollo Teleop driving.
 */

@TeleOp(name="Teleop Apollo ", group="Apollo")
public class ApolloTeleop extends OpMode{
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    private MineralVision vision;

    private ElapsedTime runtime = new ElapsedTime();

    static double speedFactor = 1;  // Speed factor
    static final double joyStickLimitPoints = 0.3;  // For better control
    static final double senderOpenLimitPoint = -7500 ; // Limit so the sender motors wont open to much, by encoder ticks.
    static final double senderCloseLimitPoint = 0 ; // Limit so the sender motors wont open to much, by encoder ticks.
    static final double convertJoystickToPosition = 10 ; //
    int mineralSenderPosition ; // Encoder positions
    int mineralSenderWantedPosition  = 0 ; // Encoder positions


    @Override
    public void init() {
        //Hardware init
        robot.init(hardwareMap);
        robot.setMineralSendMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("here", "Ready");
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
    }

    @Override
    public void loop() {
        //Controllers drive sticks inputs
        double LeftX = gamepad1.left_stick_x;
        double LeftY = -gamepad1.left_stick_y;  // The joystick goes negative when pushed forwards, so negate it.
        double RightX = gamepad1.right_stick_x;
        double RightY = -gamepad1.right_stick_y;    // The joystick goes negative when pushed forwards, so negate it
        //Mineral divider By camera
        MineralDivideByVision();

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
        if (LeftX < -joyStickLimitPoints && (Math.abs(LeftY) > joyStickLimitPoints) &&
                (RightX < -joyStickLimitPoints && Math.abs(RightY) > joyStickLimitPoints)){
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_LEFT);
            telemetry.addData("Drive", "DIAGONAL_LEFT");
        }
        else if (LeftX > joyStickLimitPoints && (Math.abs(LeftY) > joyStickLimitPoints) &&
                (RightX > joyStickLimitPoints && Math.abs(RightY) > joyStickLimitPoints)){
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_RIGHT);
            telemetry.addData("Drive", "DIAGONAL_RIGHT");
        }
        else if ((Math.abs(LeftX) > Math.abs(LeftY) && Math.abs(LeftX) > joyStickLimitPoints && Math.abs(LeftY)< 0.8) &&
                (Math.abs(RightX) > Math.abs(RightY) && Math.abs(RightX)> joyStickLimitPoints && Math.abs(RightY)< 0.8 &&
                        (RightX<0 && LeftX<0 || RightX>0 && LeftX>0 ))){
            robot.setDriveMotorsPower(LeftX*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            telemetry.addData("Drive", "Side ways");
        }
        else{
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
            robot.setDriveMotorsPower(RightY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
            telemetry.addData("Drive", "Normal");
        }
        telemetry.update();


        //Mineral graber control. Game pad 2, triggers.
        if (gamepad2.left_trigger<0 && gamepad2.right_trigger<0){
            telemetry.addData("main graber", "stop");
            robot.mineralGrab.setPower(0);
        }
        if (gamepad2.left_trigger>0){
            telemetry.addData("main graber", "grab out");
            telemetry.addData("power graber", gamepad2.left_trigger);
            robot.mineralGrab.setPower(-gamepad2.left_trigger);
        }
        else if (gamepad2.right_trigger>0){
            telemetry.addData("main graber", "grab in");
            telemetry.addData("power graber", gamepad2.right_trigger);
            robot.mineralGrab.setPower(gamepad2.right_trigger);
        }else {
            robot.mineralGrab.setPower(0);
        }

        //Mineral lift Control. Game pad 2, left stick.
        if(gamepad2.left_stick_y>0.1){
            robot.lift.setPower(1);
            //robot.mineralGrab.setPower(1);
            robot.blockMineralServo.setPosition(robot.block);
        }
        else if(gamepad2.left_stick_y<-0.1){
            robot.lift.setPower(-1);
        }
        else{
            robot.lift.setPower(0);
        }

        //Mineral sender control. Game pad 2, right stick.
        if (Math.abs(gamepad2.right_stick_y) > 0.03) {
            robot.setMineralSendMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(-gamepad2.right_stick_y > 0 && mineralSenderPosition >= senderOpenLimitPoint) {
                robot.setMineralSendPower(-gamepad2.right_stick_y);
                mineralSenderWantedPosition = robot.mineralSendRight.getCurrentPosition();
                telemetry.addData("tese", "here");
            }else if(-gamepad2.right_stick_y < 0 && mineralSenderPosition <= senderCloseLimitPoint ){
                robot.setMineralSendPower(-gamepad2.right_stick_y);
                mineralSenderWantedPosition = robot.mineralSendRight.getCurrentPosition();
                telemetry.addData("tese", "here");
            }
            else {
                robot.setMineralSendPower(0); // Run to hold still
                telemetry.addData("stop", "1");
            }
        }
        else{
            robot.setMineralSendPower(0); // Run to hold still
            telemetry.addData("stop2", "2");

        }
        mineralSenderPosition = robot.mineralSendRight.getCurrentPosition();

        telemetry.addData("Sender Encoder left", robot.mineralSendLeft.getCurrentPosition());
        telemetry.addData("Sender Encoder right", robot.mineralSendRight.getCurrentPosition());
        telemetry.addData("Sender position", mineralSenderPosition);
        telemetry.update();

        //Mineral blocker control. Game pad 2 bumper.
        if (gamepad2.right_bumper) {
            robot.blockMineralServo.setPosition(robot.block);
        } else if (gamepad2.left_bumper) {
            robot.blockMineralServo.setPosition(robot.dontBlock);
        }

        telemetry.update();
    }
    public void stop() {
        robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
        vision.disable();
    }

    //Function divides the minerals to gold and silver with servo by camera.
    public void MineralDivideByVision(){
        if(vision.goldMineralFound()== true){
            robot.mineralsDivider.setPosition(robot.dividerRight);
        }else{
            robot.mineralsDivider.setPosition(robot.dividerLeft);
        }
    }


}