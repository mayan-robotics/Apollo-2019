package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Check robot hardware.
 */

@Autonomous(name="Test: Hardware check", group="Test")
public class HardwareCheck extends RobotFunctions{
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Version", robot.Version);
        telemetry.addData("ROBOT","SHOULD NOT BE ON THE GROUND");
        telemetry.update();
        waitSeconds(3);
        telemetry.clear();

        try {
            robot.init(hardwareMap);
            telemetry.addData("Apollo","init succeeded");
            //telemetry.update();
        }catch (Exception e){
            telemetry.addData("ERROR01","INIT FAILED!   check your config");
            telemetry.update();
        }

        try {
            robot.driveLeftFront.setTargetPosition(1);
            robot.driveRightFront.setTargetPosition(1);
            robot.driveLeftBack.setTargetPosition(1);
            robot.driveRightBack.setTargetPosition(1);

            double dlf = robot.driveLeftFront.getCurrentPosition();
            double drf = robot.driveRightFront.getCurrentPosition();
            double dlb = robot.driveLeftBack.getCurrentPosition();
            double drb = robot.driveRightBack.getCurrentPosition();
            telemetry.addData("motor",dlf);
            telemetry.addData("motor",drf);
            telemetry.addData("motor",dlb);
            telemetry.addData("motor",drb);
            telemetry.update();

            if(dlf!=1 || drf!=1 || dlb!=1 || drb!=1){
                telemetry.addData("ERROR02","DRIVE MOTORS ARE NOT DETECTED!   check motors connections");
                telemetry.update();
            }else{
                telemetry.addData("DRIVE","succeed");
            }
            //telemetry.clear();

            telemetry.addData("DRIVE","succeed");
            telemetry.update();
        }catch (Exception e) {
            telemetry.addData("ERROR02","DRIVE MOTORS ARE NOT DETECTED!   check motors connections");
            telemetry.update();
        }

        try {
            double lift = robot.lift.getCurrentPosition();
            telemetry.addData("motor",lift);
            telemetry.clear();
        }catch (Exception e){
            telemetry.addData("ERROR03","DC MOTORS ARE NOT DETECTED!   check lift motor connections");
        }

        try {
            //double md = robot.mineralsDivider.getPosition();
            double b = robot.blockMineralServo.getPosition();
            //telemetry.addData("servo", md);
            telemetry.addData("servo",b);
            telemetry.clear();
        }catch (Exception e){
            telemetry.addData("ERROR04","SERVOS ARE NOT DETECTED!   check motors connections");
        }
        telemetry.addData("Apollo","Init test done");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();


        telemetry.addData("X", "Encoder drive speed test");
        telemetry.addData("Y", "Encoder drive one wheel rotation");
        telemetry.addData("B", "Gyro drive test");
        telemetry.addData("A", "Gyro drive test square");
        telemetry.addData("RB", "Drive wheels test");
        telemetry.addData("LB", "Drive Modes test");

        telemetry.update();

        while (opModeIsActive()) {
            if(gamepad1.left_trigger>0){
                telemetry.addData("Left Front",robot.driveLeftFront.getCurrentPosition());
                telemetry.addData("Left Back",robot.driveLeftBack.getCurrentPosition());
                telemetry.addData("Right Front",robot.driveRightFront.getCurrentPosition());
                telemetry.addData("Right Back",robot.driveRightBack.getCurrentPosition());
                telemetry.update();
            }


            if (gamepad1.x)
            {
                //Encoder drive speed test
                telemetry.clear();
                telemetry.addData("To STOP press","right trigger");
                telemetry.update();
                while (opModeIsActive() && gamepad1.right_trigger < 0.2)
                {
                    robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.setDriveMotorsPower(0.1, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                    telemetry.addData("Left Front",robot.driveLeftFront.getCurrentPosition());
                    telemetry.addData("Left Back",robot.driveLeftBack.getCurrentPosition());
                    telemetry.addData("Right Front",robot.driveRightFront.getCurrentPosition());
                    telemetry.addData("Right Back",robot.driveRightBack.getCurrentPosition());
                    telemetry.update();
                }
                robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                telemetry.addData("Test","Stopped");
                telemetry.update();
            } else if (gamepad1.y)
            {
                //Encoder drive one wheel rotation
                telemetry.clear();
                while (opModeIsActive() && gamepad1.right_trigger < 0.2) {
                    int newLeftFrontTarget = robot.driveLeftFront.getCurrentPosition() + (int) (robot.COUNTS_PER_INCH);
                    int newLeftBackTarget = robot.driveLeftBack.getCurrentPosition() + (int) (robot.COUNTS_PER_INCH);
                    int newRightFrontTarget = robot.driveRightFront.getCurrentPosition() + (int) (robot.COUNTS_PER_INCH);
                    int newRightBackTarget = robot.driveRightBack.getCurrentPosition() + (int) (robot.COUNTS_PER_INCH);

                    robot.driveLeftFront.setTargetPosition(newLeftFrontTarget);
                    robot.driveLeftBack.setTargetPosition(newLeftBackTarget);
                    robot.driveRightFront.setTargetPosition(newRightFrontTarget);
                    robot.driveRightBack.setTargetPosition(newRightBackTarget);

                    robot.setDriveMotorsPower(0.5, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                    robot.setDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addData("Apollo","finished test");
                    telemetry.update();
                    break;
                }
                telemetry.addData("Test","Stopped");
                telemetry.update();
            } else if (gamepad1.b)
            {
                //Gyro drive test
                telemetry.clear();
                //while (opModeIsActive() && gamepad1.right_trigger < 0.2)
                //{
                try{
                    gyroDrive(0.2, 130, 0);
                }catch (InterruptedException e) { }
                //}
                telemetry.addData("Test","Stopped");
                telemetry.update();
            } else if (gamepad1.a)
            {
                //Gyro drive square
                telemetry.clear();
                while (opModeIsActive() && gamepad1.right_trigger < 0.2)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        try{
                        driveByGyro(0.7, 150, 90 * i);
                        waitSeconds(1);
                        turnByGyro(TURN_SPEED, 90 * (i + 1));
                        waitSeconds(2);
                        }catch (InterruptedException e) { }
                    }
                    telemetry.addData("Apollo","finished test");
                    telemetry.update();
                    waitSeconds(1);
                    break;
                }
                telemetry.clear();
                telemetry.addData("Test","Stopped");
                telemetry.update();
            }
            else if (gamepad1.right_bumper)
            {
                //Drive wheels test
                telemetry.clear();
                while (opModeIsActive() && gamepad1.right_trigger < 0.2)
                {
                    robot.driveLeftFront.setPower(0.2);
                    waitSeconds(3);
                    robot.driveLeftFront.setPower(0);
                    if(gamepad1.right_trigger < 0.2){break;}

                    robot.driveLeftBack.setPower(0.2);
                    waitSeconds(3);
                    robot.driveLeftBack.setPower(0);
                    if(gamepad1.right_trigger < 0.2){break;}

                    robot.driveRightBack.setPower(0.2);
                    waitSeconds(3);
                    robot.driveRightBack.setPower(0);
                    if(gamepad1.right_trigger < 0.2){break;}

                    robot.driveRightFront.setPower(0.2);
                    waitSeconds(3);
                    robot.driveRightFront.setPower(0);
                    if(gamepad1.right_trigger < 0.2){break;}
                    telemetry.addData("Apollo","finished test");
                    telemetry.update();
                    break;
                }
                telemetry.clear();
                telemetry.addData("Test","Stopped");
                telemetry.update();
            }
            else if (gamepad1.left_bumper)
            {
                //Drive Modes test
                telemetry.clear();
                while (opModeIsActive() && gamepad1.right_trigger < 0.2)
                {
                    robot.setDriveMotorsPower(0.2, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                    waitSeconds(3);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                    waitSeconds(1);
                    robot.setDriveMotorsPower(-0.2, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                    waitSeconds(3);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                    if(gamepad1.right_trigger < 0.2){break;}
                    waitSeconds(2);


                    robot.setDriveMotorsPower(0.2, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
                    waitSeconds(3);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
                    waitSeconds(1);
                    robot.setDriveMotorsPower(-0.2, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
                    waitSeconds(3);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
                    if(gamepad1.right_trigger < 0.2){break;}
                    waitSeconds(2);

                    robot.setDriveMotorsPower(0.2, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
                    waitSeconds(3);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
                    waitSeconds(1);
                    robot.setDriveMotorsPower(-0.2, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
                    waitSeconds(3);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
                    if(gamepad1.right_trigger < 0.2){break;}
                    waitSeconds(2);

                    robot.setDriveMotorsPower(0.2, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                    waitSeconds(5);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                    waitSeconds(2);
                    robot.setDriveMotorsPower(-0.2, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                    waitSeconds(5);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                    if(gamepad1.right_trigger < 0.2){break;}
                    telemetry.addData("Apollo","finished test");
                    telemetry.update();
                    break;
                }
                telemetry.clear();
                telemetry.addData("Test","Stopped");
                telemetry.update();
            }
            else if(gamepad1.dpad_up){
                try {
                    encoderSideWaysDrive(1,30);
                }catch (InterruptedException e) { }
            }
            else {
                telemetry.addData("X", "Encoder drive speed test");
                telemetry.addData("Y", "Encoder drive one wheel rotation");
                telemetry.addData("B", "Gyro drive test");
                telemetry.addData("A", "Gyro drive test square");
                telemetry.addData("RB", "Drive wheels test");
                telemetry.addData("LB", "Drive Modes test");
            }
            telemetry.update();
        }
    }


    public void waitSeconds(double seconds){
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {}
    }

}

