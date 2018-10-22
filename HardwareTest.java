package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Apollo 2019
 * This class is used to define all the specific hardware for Apollo's robot.
 */

public class HardwareTest {

        /* Public OpMode members. */
        public DcMotor  driveLeftFront = null;
        public DcMotor  driveLeftBack = null;
        public DcMotor  driveRightFront = null;
        public DcMotor  driveRightBack = null;
        public DcMotor  mainGraber = null;
        public DcMotor  graberPusher = null;
        public DcMotor  lift = null;
        public DcMotor  mineralsUp = null;
        public Servo    blockMineralServo = null;
        public Servo    mineralsDivider = null;

        //BNO055IMU imu;

        //Declaration of the drive motor types.
        public enum DRIVE_MOTOR_TYPES {
                LEFT,
                RIGHT,
                SIDE_WAYS,
                DIAGONAL_RIGHT,
                DIAGONAL_LEFT,
                ALL
        }

        static final double block = 0;
        static final double dontBlock = 0.2;

        /* local OpMode members. */
        HardwareMap hwMap  =  null;

        /* Constructor */
        public HardwareTest(){

        }

        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
                // Save reference to Hardware map
                hwMap = ahwMap;

                // Define and Initialize Motors
                driveLeftFront  = hwMap.get(DcMotor.class, "dlf");
                driveRightFront  = hwMap.get(DcMotor.class, "drf");
                driveLeftBack  = hwMap.get(DcMotor.class, "dlb");
                driveRightBack  = hwMap.get(DcMotor.class, "drb");
                //lift  = hwMap.get(DcMotor.class, "l");
                mainGraber  = hwMap.get(DcMotor.class, "m");
                graberPusher  = hwMap.get(DcMotor.class, "gp");
                //mineralsUp  = hwMap.get(DcMotor.class, "mt");
                blockMineralServo  = hwMap.get(Servo.class, "bs");
                mineralsDivider  = hwMap.get(Servo.class, "md");
                mineralsDivider.setPosition(0.5);
                // Set all motors to zero power
                //driveRightFront.setPower(0);
                //driveLeftFront.setPower(0);
                setDriveMotorsPower(0, DRIVE_MOTOR_TYPES.ALL);
                mainGraber.setPower(0);
                graberPusher.setPower(0);
                blockMineralServo.setPosition(block);
                //graberServo.setPosition(0);
                //mineralsUp.setPower(0);
                driveLeftFront.setDirection(DcMotor.Direction.REVERSE);
                driveRightFront.setDirection(DcMotor.Direction.REVERSE);


                // Set all motors to run without encoders.
                //driveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //driveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setDriveMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                // May want to use RUN_USING_ENCODERS if encoders are installed.


                // Define and initialize ALL installed servos.

        }


        //Function to set the power to all the drive motors.
        public void setDriveMotorsPower(double power, DRIVE_MOTOR_TYPES driverMotorType){
                switch (driverMotorType){
                        case LEFT:
                                driveLeftFront.setPower(power);
                                driveLeftBack.setPower(power);
                                break;
                        case RIGHT:
                                driveRightFront.setPower(power);
                                driveRightBack.setPower(power);
                                break;
                        case SIDE_WAYS:
                                driveLeftBack.setPower(-power);
                                driveLeftFront.setPower(power);
                                driveRightBack.setPower(power);
                                driveRightFront.setPower(-power);
                                break;
                        case DIAGONAL_LEFT:
                                driveRightFront.setPower(power);
                                driveLeftBack.setPower(power);
                                break;
                        case DIAGONAL_RIGHT:
                                driveLeftFront.setPower(power);
                                driveRightBack.setPower(power);
                                break;

                        case ALL:
                        default:
                                driveLeftFront.setPower(power);
                                driveLeftBack.setPower(power);
                                driveRightFront.setPower(power);
                                driveRightBack.setPower(power);
                                break;


                }

        }

        //Function to set the position to all the drive motors.
        public void setDriveMotorsPosition(int Target, DRIVE_MOTOR_TYPES driverMotorType){
                switch (driverMotorType){
                        case LEFT:
                                driveLeftFront.setTargetPosition(Target);
                                driveLeftBack.setTargetPosition(Target);
                                break;
                        case RIGHT:
                                driveRightFront.setTargetPosition(Target);
                                driveRightBack.setTargetPosition(Target);
                                break;
                        case SIDE_WAYS:
                                driveLeftBack.setTargetPosition(-Target);
                                driveLeftFront.setTargetPosition(Target);
                                driveRightBack.setTargetPosition(Target);
                                driveRightFront.setTargetPosition(-Target);
                                break;
                        case ALL:
                        default:
                                driveLeftFront.setTargetPosition(Target);
                                driveLeftBack.setTargetPosition(Target);
                                driveRightFront.setTargetPosition(Target);
                                driveRightBack.setTargetPosition(Target);
                                break;

                }

        }

        //Function to set the run mode for all the drive motors.
        public void setDriveMotorsMode(DcMotor.RunMode runMode) {
                driveLeftFront.setMode(runMode);
                driveLeftBack.setMode(runMode);
                driveRightFront.setMode(runMode);
                driveRightBack.setMode(runMode);

        }


}
