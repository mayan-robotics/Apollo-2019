package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Apollo Teleop driving.
 */

@TeleOp(name="Apollo Teleop", group="Apollo")
public class TeleopApollo extends OpMode{
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware

    // Declartion of the drive state.
    private enum DRIVE_CONTROL_STATE {
        STRAIGHT,
        SIDE_WAY_LEFT,
        SIDE_WAY_RIGHT
    }
    // Controller Parameters
    private class ControlParameters {
        double LeftPower;
        double RightPower;
        DRIVE_CONTROL_STATE driveControlState;
    }

    @Override
    public void init() {
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Apollo", "Ready");
    }

    @Override
    public void loop() {
        if(ControllerWay().driveControlState == DRIVE_CONTROL_STATE.STRAIGHT){
            robot.setDriveMotorsPower(ControllerWay().LeftPower, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
            robot.setDriveMotorsPower(ControllerWay().RightPower, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
        }
        else if (ControllerWay().driveControlState == DRIVE_CONTROL_STATE.SIDE_WAY_LEFT){
            robot.setDriveMotorsPower(ControllerWay().LeftPower, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAY_LEFT);
        }
        else if (ControllerWay().driveControlState == DRIVE_CONTROL_STATE.SIDE_WAY_RIGHT){
            robot.setDriveMotorsPower(ControllerWay().RightPower, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAY_RIGHT);
        }

    }

    // This function declares the drive state of the robot based on the controller.
    // Side way drive should activate only when both joy sticks are pushed to the same side.
    public ControlParameters ControllerWay() {
        final double speedFactor = 0.5; // Decrease the speed factor
        final int LeftLimitPoint = -50;
        final int RightLimitPoint = 50;
        // The joystick goes negative when pushed forwards, so negate it)
        double LeftX = -gamepad1.left_stick_x;
        double LeftY = -gamepad1.left_stick_y;
        double RightX = -gamepad1.right_stick_x;
        double RightY = -gamepad1.right_stick_y;

        ControlParameters driveControlParameters = null;

        if ((Math.abs(LeftX) < Math.abs(LeftY) && LeftX < LeftLimitPoint) &&
                (Math.abs(RightX) < Math.abs(RightY) && RightX < LeftLimitPoint)){

            driveControlParameters.driveControlState=DRIVE_CONTROL_STATE.SIDE_WAY_LEFT;
            driveControlParameters.LeftPower = LeftX*speedFactor ;
        }
        else if ((Math.abs(LeftX) > Math.abs(LeftY) && LeftX > RightLimitPoint) &&
                    (Math.abs(RightX) > Math.abs(RightY) && RightX > RightLimitPoint)) {
            driveControlParameters.driveControlState=DRIVE_CONTROL_STATE.SIDE_WAY_RIGHT;
            driveControlParameters.RightPower = RightX*speedFactor ;
        }
        else{
            driveControlParameters.driveControlState=DRIVE_CONTROL_STATE.STRAIGHT;
            driveControlParameters.LeftPower =LeftY*speedFactor ;
            driveControlParameters.RightPower =RightY*speedFactor ;
        }
        return driveControlParameters;

    }

}