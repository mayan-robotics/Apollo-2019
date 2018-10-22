package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

/**
 * Apollo Teleop driving.
 */

@TeleOp(name="Teleop test", group="Apollo")
public class TeleopTest extends OpMode{
    HardwareTeleopTest robot = new HardwareTeleopTest(); // use Apollo's hardware
    private MineralVision vision;

    static double speedFactor = 1; // Decrease the speed factor
    //double servo = 0; // Decrease the speed factor
    static final double limitPoint = 0.4;
    static final double dividerMiddle = 0.5;
    static final double dividerLeft = 0.4;
    static final double dividerRight = 0.6;


    @Override
    public void init() {
        robot.init(hardwareMap);
        vision = new MineralVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        vision.setShowCountours(false);
        // start the vision system
        vision.enable();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Apollo", "Ready");
        telemetry.update();
        vision.setShowCountours(true);
    }

    @Override
    public void loop() {
        double LeftX = gamepad1.left_stick_x;
        double LeftY = -gamepad1.left_stick_y;
        double RightX = gamepad1.right_stick_x;
        double RightY = gamepad1.right_stick_y;
        vision();

        //robot.setDriveMotorsPower(LeftY * speedFactor, HardwareTeleopTest.DRIVE_MOTOR_TYPES.LEFT);
        //robot.setDriveMotorsPower(RightY * speedFactor, HardwareTeleopTest.DRIVE_MOTOR_TYPES.RIGHT);

        if (gamepad1.x) {
            speedFactor= 1;
        } else if (gamepad1.y) {
            speedFactor= 0.5;
        }
        telemetry.addData("left x", LeftX );
        telemetry.addData("left y", LeftY );
        telemetry.addData("right x", RightX );
        telemetry.addData("right y", RightY );


        if ((Math.abs(LeftX) > Math.abs(LeftY) && Math.abs(LeftX) > limitPoint) &&
                (Math.abs(RightX) > Math.abs(RightY) && Math.abs(RightX)> limitPoint)){
            robot.setDriveMotorsPower(LeftX*speedFactor, HardwareTeleopTest.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            telemetry.addData("Drive", "Side ways");
        }
        else if (LeftX < -limitPoint && (Math.abs(LeftY) > limitPoint) &&
                (RightX < -limitPoint && Math.abs(RightY) > limitPoint)){
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareTeleopTest.DRIVE_MOTOR_TYPES.DIAGONAL_LEFT);
            telemetry.addData("Drive", "DIAGONAL_LEFT");
        }
        else if (LeftX > limitPoint && (Math.abs(LeftY) > limitPoint) &&
                (RightX > limitPoint && Math.abs(RightY) > limitPoint)){
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareTeleopTest.DRIVE_MOTOR_TYPES.DIAGONAL_RIGHT);
            telemetry.addData("Drive", "DIAGONAL_RIGHT");
        }
        else{
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareTeleopTest.DRIVE_MOTOR_TYPES.LEFT);
            robot.setDriveMotorsPower(RightY*speedFactor, HardwareTeleopTest.DRIVE_MOTOR_TYPES.RIGHT);
            telemetry.addData("Drive", "Normal");
        }


        if (-gamepad2.right_stick_y>0.3) {
            robot.blockMineralServo.setPosition(robot.block);
        } else if (-gamepad2.right_stick_y<-0.3) {
            robot.blockMineralServo.setPosition(robot.dontBlock);
        }



        if (gamepad2.left_trigger<0 && gamepad2.right_trigger<0){
            telemetry.addData("main graber", "stop");
            robot.mainGraber.setPower(0);
        }
        if (gamepad2.left_trigger>0){
            telemetry.addData("main graber", "grab out");
            robot.mainGraber.setPower(-gamepad2.left_trigger);
        }
        else if (gamepad2.right_trigger>0){
            telemetry.addData("main graber", "grab in");
            robot.mainGraber.setPower(gamepad2.right_trigger);
        }else {
            telemetry.addData("main graber", "stop");
            robot.mainGraber.setPower(0);
        }

        //if(gamepad2.left_bumper && gamepad2.right_bumper) {
        //    robot.graberPusher.setPower(0);
        //    telemetry.addData("push", "stop");
        //}
        if(gamepad2.left_stick_y>0.1){
            robot.graberPusher.setPower(1);
            robot.mainGraber.setPower(1);
            robot.blockMineralServo.setPosition(robot.block);
            telemetry.addData("push", "out");
        }
        else if(gamepad2.left_stick_y<-0.1){
            robot.graberPusher.setPower(-1);
            telemetry.addData("push", "in");
        }
        else{
            robot.graberPusher.setPower(0);
            telemetry.addData("push", "stop");
        }
            telemetry.update();

/*
        if (gamepad1.left_trigger==0 || gamepad1.right_trigger==0){
            if (gamepad1.left_trigger>0){
                telemetry.addData("main graber", "grab in");
                robot.mainGraber.setPower(-gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger>0) {
                telemetry.addData("main graber", "grab out");
                robot.mainGraber.setPower(gamepad1.right_trigger);
            }
        }else {
            telemetry.addData("main graber", "stop");
            robot.mainGraber.setPower(0);
        }


        if(!gamepad1.left_bumper || !gamepad1.right_bumper) {
            if (gamepad1.left_bumper) {
                robot.graberPusher.setPower(0.5);
                telemetry.addData("push", "out");
            } else if (gamepad1.right_bumper) {
                robot.graberPusher.setPower(-0.5);
                telemetry.addData("push", "in");
            }
        }
        else{
            robot.graberPusher.setPower(0);
            telemetry.addData("push", "stop");
        }
      */

    }
    public void stop() {
        robot.setDriveMotorsPower(0, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);
        vision.disable();
    }
    public void vision(){
            if(vision.goldMineralFound()== true){
                telemetry.addData("Apollo","found a gold mineral");
                robot.mineralsDivider.setPosition(dividerLeft);
                //waitSeconds(1);
            }else{
                telemetry.addData("Apollo"," did not found a gold mineral");
                robot.mineralsDivider.setPosition(dividerRight);
            }
            telemetry.update();
    }

}

/*
        if ((Math.abs(LeftX) > Math.abs(LeftY) && Math.abs(LeftX) > limitPoint) &&
                (Math.abs(RightX) > Math.abs(RightY) && Math.abs(RightX)> limitPoint)){
            robot.setDriveMotorsPower(LeftX*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
        }
        else if (LeftX < -limitPoint && (Math.abs(LeftY) > limitPoint) &&
                (RightX < -limitPoint && Math.abs(RightY) < limitPoint)){
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_LEFT);
        }
        else if (LeftX > limitPoint && (Math.abs(LeftY) > limitPoint) &&
                (RightX > limitPoint && Math.abs(RightY) < limitPoint)){
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_RIGHT);
        }
        else{
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
            robot.setDriveMotorsPower(RightY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
        }
 */

