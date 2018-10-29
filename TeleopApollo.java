package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

/**
 * Apollo Teleop driving.
 */

@TeleOp(name="Apollo Teleop", group="Apollo")
public class TeleopApollo extends OpMode{
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    private MineralVision vision;

    static double speedFactor = 1; // Decrease the speed factor
    //double servo = 0; // Decrease the speed factor
    static final double limitPoint = 0.4;



    @Override
    public void init() {
        robot.init(hardwareMap);
        vision = new MineralVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        vision.enable();
        vision.setShowCountours(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Apollo", "Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        //Mineral divider By camera
        MineralDivideByVision();
        //Controllers drive sticks inputs
        double LeftX = gamepad1.left_stick_x;
        double LeftY = -gamepad1.left_stick_y;
        double RightX = gamepad1.right_stick_x;
        double RightY = gamepad1.right_stick_y;

        //Drive speed control
        if (gamepad1.right_stick_button && gamepad1.left_stick_button){
            speedFactor = 0.5;
        }
        else{
            speedFactor = 1;
        }

        // Drive modes controls
        if (LeftX < -limitPoint && (Math.abs(LeftY) > limitPoint) &&
                (RightX < -limitPoint && Math.abs(RightY) < limitPoint)){
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_LEFT);
            telemetry.addData("Drive", "Diagonal LEFT");
        }
        else if (LeftX > limitPoint && (Math.abs(LeftY) > limitPoint) &&
                (RightX > limitPoint && Math.abs(RightY) < limitPoint)){
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_RIGHT);
            telemetry.addData("Drive", "Diagonal Right");
        }
        else if ((Math.abs(LeftX) > Math.abs(LeftY) && Math.abs(LeftX) > limitPoint) &&
                (Math.abs(RightX) > Math.abs(RightY) && Math.abs(RightX)> limitPoint)){
            telemetry.addData("Drive", "Side Ways");
            robot.setDriveMotorsPower(LeftX*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
        }
        else{
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
            robot.setDriveMotorsPower(RightY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
            telemetry.addData("Drive", "normal");
        }


        //Mineral graber control
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

        //Mineral lift Control
        if(gamepad2.left_stick_y>0.1){
            robot.graberPusher.setPower(1);
            robot.mineralGrab.setPower(1);
            robot.blockMineralServo.setPosition(robot.block);
        }
        else if(gamepad2.left_stick_y<-0.1){
            robot.graberPusher.setPower(-1);
        }
        else{
            robot.graberPusher.setPower(0);
        }


        //Mineral blocker control
        if (-gamepad2.right_stick_y>0.3) {
            robot.blockMineralServo.setPosition(robot.block);
        } else if (-gamepad2.right_stick_y<-0.3) {
            robot.blockMineralServo.setPosition(robot.dontBlock);
        }

        telemetry.update();
    }
    public void stop() {
        robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
        vision.disable();
    }

    public void MineralDivideByVision(){
        if(vision.goldMineralFound()== true){
            robot.mineralsDivider.setPosition(robot.dividerLeft);
        }else{
            robot.mineralsDivider.setPosition(robot.dividerRight);
        }
    }

}