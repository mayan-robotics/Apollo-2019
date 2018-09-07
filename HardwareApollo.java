package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Apollo 2019
 * This class is used to define all the specific hardware for Apollo's robot.
 */

public class HardwareApollo {

        /* Public OpMode members. */
        public DcMotor  driveLeftFront = null;
        public DcMotor  driveLeftBack = null;
        public DcMotor  driveRightFront = null;
        public DcMotor  driveRightBack = null;
        BNO055IMU imu;

        //Declartion of the drive motor types.
        public enum DRIVE_MOTOR_TYPES {
            LEFT,
            RIGHT,
            SIDE_WAY_LEFT_DRIVE,
            SIDE_WAY_RIGHT_DRIVE,
            ALL
        }

        /* local OpMode members. */
        HardwareMap hwMap  =  null;

        /* Constructor */
        public HardwareApollo(){

        }

        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize Motors
            driveLeftFront  = hwMap.get(DcMotor.class, "");
            driveLeftBack  = hwMap.get(DcMotor.class, "");
            driveRightFront  = hwMap.get(DcMotor.class, "");
            driveRightBack  = hwMap.get(DcMotor.class, "");

            // Set all motors to zero power
            setDriveMotorsPower(0, DRIVE_MOTOR_TYPES.ALL);

            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            // Set all motors to run without encoders.
            setDriveMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // May want to use RUN_USING_ENCODERS if encoders are installed.


            // Define and initialize ALL installed servos.

        }


        //Function to set the power to all the motors.
        public void setDriveMotorsPower(double Speed, DRIVE_MOTOR_TYPES driverMotorType){
            switch (driverMotorType){
                case LEFT:
                    driveLeftFront.setPower(Speed);
                    driveLeftBack.setPower(Speed);
                    break;
                case RIGHT:
                    driveRightFront.setPower(Speed);
                    driveRightBack.setPower(Speed);
                    break;
                case SIDE_WAY_LEFT_DRIVE:
                    driveLeftBack.setPower(-Speed);
                    driveLeftFront.setPower(Speed);
                    driveRightBack.setPower(Speed);
                    driveRightFront.setPower(-Speed);
                    break;
                case SIDE_WAY_RIGHT_DRIVE:
                    driveLeftBack.setPower(Speed);
                    driveLeftFront.setPower(-Speed);
                    driveRightBack.setPower(-Speed);
                    driveRightFront.setPower(Speed);
                    break;

                case ALL:
                    default:
                    driveLeftFront.setPower(Speed);
                    driveLeftBack.setPower(Speed);
                    driveRightFront.setPower(Speed);
                    driveRightBack.setPower(Speed);
                    break;

            }

        }


        //Function to set the position to all the motors.
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
                case SIDE_WAY_LEFT_DRIVE:
                    driveLeftBack.setPower(-Target);
                    driveLeftFront.setPower(Target);
                    driveRightBack.setPower(Target);
                    driveRightFront.setPower(-Target);
                    break;
                case SIDE_WAY_RIGHT_DRIVE:
                    driveLeftBack.setPower(Target);
                    driveLeftFront.setPower(-Target);
                    driveRightBack.setPower(-Target);
                    driveRightFront.setPower(Target);
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

        //Function to set the mode run using encoder for all the motors.
        public void setDriveMotorsMode(DcMotor.RunMode runMode) {
            driveLeftFront.setMode(runMode);
            driveLeftBack.setMode(runMode);
            driveRightFront.setMode(runMode);
            driveRightBack.setMode(runMode);

        }



}
