package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

/**
 * Apollo Teleop driving.
 */

@TeleOp(name="Teleop Apollo ", group="Apollo")
public class ApolloTeleop extends OpMode{
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    private MineralVision vision;

    static double speedFactor = 1;  // Speed factor
    static final double joyStickLimitPoints = 0.3;  // for better control

    @Override
    public void init() {
        //Hardware init
        robot.init(hardwareMap);
        vision = new MineralVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        vision.enable();
        vision.setShowCountours(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Apollo", "Ready");
        telemetry.addData("Version", "1.11.21");
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

        //Drive speed control. Game pad 1, stick buttons.
        if (gamepad1.right_stick_button && gamepad1.left_stick_button){
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
            robot.mineralGrab.setPower(-gamepad2.left_trigger);
        }
        else if (gamepad2.right_trigger>0){
            telemetry.addData("main graber", "grab in");
            robot.mineralGrab.setPower(gamepad2.right_trigger);
        }else {
            telemetry.addData("main graber", "stop");
            robot.mineralGrab.setPower(0);
        }

        //Mineral lift Control. Game pad 2, left stick.
        if(gamepad2.left_stick_y>0.1){
            robot.lift.setPower(1);
            robot.mineralGrab.setPower(1);
            robot.blockMineralServo.setPosition(robot.block);
        }
        else if(gamepad2.left_stick_y<-0.1){
            robot.lift.setPower(-1);
        }
        else{
            robot.lift.setPower(0);
        }

        //Mineral sender control. Game pad 2, right stick.
        if (Math.abs(gamepad2.right_stick_y) > 0.3) {
            robot.setMineralSendPower(-gamepad2.right_stick_y);
        }
        else{
            robot.setMineralSendPower(0.03); // Run to hold still
        }

        //Mineral blocker control. Game pad 2 bumper.
        if (gamepad2.right_bumper) {
            robot.blockMineralServo.setPosition(robot.block);
        } else if (gamepad2.right_bumper) {
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