package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Check robot hardware.
 */

@Autonomous(name="Hardware check", group="Apollo")
public class HardwareCheck extends AutoMain{
    HardwareTeleopTest robot = new HardwareTeleopTest(); // use Apollo's hardware
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        try {
            robot.init(hardwareMap);
            telemetry.addData("Apollo","init succeed");
            //telemetry.update();
        }catch (Exception e){
            telemetry.addData("ERROR01","INIT FAILED!   check your config");
            //telemetry.update();
        }

        // Send telemetry message to signify robot waiting;
        try {
            double dlf = robot.driveLeftFront.getCurrentPosition();
            double drf = robot.driveRightFront.getCurrentPosition();
            double dlb = robot.driveLeftBack.getCurrentPosition();
            double drb = robot.driveRightBack.getCurrentPosition();
            telemetry.addData("motor",dlf);
            telemetry.addData("motor",drf);
            telemetry.addData("motor",dlb);
            telemetry.addData("motor",drb);
            telemetry.update();
            telemetry.clear();

            telemetry.addData("DRIVE","succeed");
            telemetry.addData("Apollo", "Ready");
            //telemetry.update();
        }catch (Exception e) {
            telemetry.addData("ERROR02","DRIVE MOTORS ARE NOT DETECTED!   check motors connections");
            //telemetry.update();
        }

        try {
            double m = robot.mainGraber.getCurrentPosition();
            double p = robot.graberPusher.getCurrentPosition();
            telemetry.addData("motor",m);
            telemetry.addData("motor",p);
            telemetry.clear();
        }catch (Exception e){
            telemetry.addData("ERROR03","DC MOTORS ARE NOT DETECTED!   check motors connections");
        }

        try {
            double md = robot.mineralsDivider.getPosition();
            double b = robot.blockMineralServo.getPosition();
            telemetry.addData("servo", md);
            telemetry.addData("servo",b);
            telemetry.clear();
        }catch (Exception e){
            telemetry.addData("ERROR04","SERVOS ARE NOT DETECTED!   check motors connections");
        }

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        try {
            robot.setDriveMotorsPower(0.1, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);
            waitSeconds(2);
            robot.setDriveMotorsPower(-0.1, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);
            waitSeconds(2);
            robot.setDriveMotorsPower(1, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);
            waitSeconds(1);
            robot.setDriveMotorsPower(-1, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);
            waitSeconds(1);
            robot.setDriveMotorsPower(0, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);
            waitSeconds(2);
            robot.setDriveMotorsPower(1, HardwareTeleopTest.DRIVE_MOTOR_TYPES.RIGHT);
            waitSeconds(2);
            robot.setDriveMotorsPower(-1, HardwareTeleopTest.DRIVE_MOTOR_TYPES.RIGHT);
            waitSeconds(2);
            robot.setDriveMotorsPower(0, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);
            waitSeconds(2);
            robot.setDriveMotorsPower(1, HardwareTeleopTest.DRIVE_MOTOR_TYPES.LEFT);
            waitSeconds(2);
            robot.setDriveMotorsPower(-1, HardwareTeleopTest.DRIVE_MOTOR_TYPES.LEFT);
            waitSeconds(2);
            robot.setDriveMotorsPower(0, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);
            waitSeconds(2);
            robot.setDriveMotorsPower(1, HardwareTeleopTest.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            waitSeconds(2);
            robot.setDriveMotorsPower(-1, HardwareTeleopTest.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            waitSeconds(2);
            robot.setDriveMotorsPower(0, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);
            waitSeconds(1);
            telemetry.addData("Apollo","Drive motors succeed");
            //telemetry.update();
        }catch (Exception e) {
            telemetry.addData("ERROR","FAILED!");
            //telemetry.update();
        }
        telemetry.update();
        waitSeconds(999);

    }
    public void waitSeconds(double seconds) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
        }
    }
}