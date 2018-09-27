package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
        public DcMotor  mineralGrab = null;
        public Servo    mineralsDivider = null;
        public ColorSensor mineralsColor = null;

        BNO055IMU imu;

        //Declaration of the drive motor types.
        public enum DRIVE_MOTOR_TYPES {
            LEFT,
            RIGHT,
            SIDE_WAY_LEFT_DRIVE,
            SIDE_WAY_RIGHT_DRIVE,
            ALL
        }

        //Declaration of Minerals.
        public enum TYPE_MINERALS {
            GOLD,
            SILVER,
        }


        /* local OpMode members. */
        HardwareMap hwMap  =  null;

        /* Constructor */
        public HardwareApollo(){

        }

        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Servo mineral divider positions
            double mineralsDividerMiddle = 0.5;
            double mineralsDividerLeft = 0.3;
            double mineralsDividerRight = 0.7;


            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize ALL Motors
            driveLeftFront  = hwMap.get(DcMotor.class, "");
            driveLeftBack  = hwMap.get(DcMotor.class, "");
            driveRightFront  = hwMap.get(DcMotor.class, "");
            driveRightBack  = hwMap.get(DcMotor.class, "");
            mineralGrab  = hwMap.get(DcMotor.class, "");

            // Define and initialize ALL servos.
            mineralsDivider  = hwMap.get(Servo.class, "");

            // Define and initialize ALL sensors
            mineralsColor  = hwMap.get(ColorSensor.class, "");

            // Set all motors to zero power
            setDriveMotorsPower(0, DRIVE_MOTOR_TYPES.ALL);
            mineralGrab.setPower(0);

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



        }


        //Function to set the power to all the drive motors.
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

        //Function to set the run mode for all the drive motors.
        public void setDriveMotorsMode(DcMotor.RunMode runMode) {
            driveLeftFront.setMode(runMode);
            driveLeftBack.setMode(runMode);
            driveRightFront.setMode(runMode);
            driveRightBack.setMode(runMode);

        }

        //Function to set the power to all the minerals motors.
        public void runAllMineralsMotor(double power) {
            mineralGrab.setPower(power);

        }

        //Function to set the run mode for all the minerals motors.
        public void setModeAllMineralsMotor(DcMotor.RunMode runMode) {
            mineralGrab.setMode(runMode);

        }
}
