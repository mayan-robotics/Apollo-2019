package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

/**
 * Apollo Teleop driving.
 */

@TeleOp(name="Teleop2 test", group="Apollo")
public class TeleopApollo2 extends OpMode{
    HardwareTeleopTest robot = new HardwareTeleopTest(); // use Apollo's hardware

    static double speedFactor = 1; // Decrease the speed factor
    //double servo = 0; // Decrease the speed factor
    static final double limitPoint = 0.4;
    static final double dividerMiddle = 0.5;
    static final double dividerLeft = 0.4;
    static final double dividerRight = 0.6;


    @Override
    public void init() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Apollo", "Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        double LeftX = gamepad1.left_stick_x;
        double LeftY = -gamepad1.left_stick_y;
        double RightX = gamepad1.right_stick_x;
        double RightY = gamepad1.right_stick_y;



        if (Math.abs(LeftY) >= 0.1 && Math.abs(LeftX) <= 0.1 && Math.abs(RightX) <=0.1 && Math.abs(RightY) <= 0.1){
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);
        }
        else if (Math.abs(LeftX) >= 0.1 && Math.abs(LeftY) <= 0.1 && Math.abs(RightX) <= 0.1 && Math.abs(RightY) <= 0.1){
            robot.setDriveMotorsPower(LeftX*speedFactor, HardwareTeleopTest.DRIVE_MOTOR_TYPES.SIDE_WAYS);
        }
        else if (LeftX > 0 && LeftY < 0 || LeftX < 0 && LeftY > 0){
            robot.setDriveMotorsPower(LeftX, HardwareTeleopTest.DRIVE_MOTOR_TYPES.DIAGONAL_LEFT);
        }
        else if (Math.abs(LeftX) > 0 && Math.abs(LeftY) > 0 || Math.abs(LeftX) < 0 && Math.abs(LeftY) < 0) {
            robot.setDriveMotorsPower(LeftY, HardwareTeleopTest.DRIVE_MOTOR_TYPES.DIAGONAL_RIGHT);
        }

        if (Math.abs(RightX) > 0){
            robot.setDriveMotorsPower(RightX, HardwareTeleopTest.DRIVE_MOTOR_TYPES.RIGHT);
            robot.setDriveMotorsPower(-RightX, HardwareTeleopTest.DRIVE_MOTOR_TYPES.LEFT);
        }


    }
    public void stop() {
        robot.setDriveMotorsPower(0, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);
    }


}


