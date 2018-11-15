package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

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
    public DcMotor  lift = null;
    public DcMotor  mineralSend = null;

    public Servo    blockMineralServo = null;
    public Servo    mineralsDivider = null;
    BNO055IMU imu;

    //Declaration of the drive motor types.
    public enum DRIVE_MOTOR_TYPES {
        LEFT,
        RIGHT,
        SIDE_WAYS,
        DIAGONAL_RIGHT,
        DIAGONAL_LEFT,
        ALL
    }

    //Declaration of Minerals.
    public enum TYPE_MINERALS {
        GOLD,
        SILVER,
    }

    // Encoder
    static final double     COUNTS_PER_MOTOR_REV    = 280 ;    // PPR for NeverRest 40
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // Gear 2:1 , two motor cycle is to one wheel cycle
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);

    // Servo mineral divider positions
    static final double dividerMiddle = 0.5;
    static final double dividerLeft = 0.45;
    static final double dividerRight = 0.55;

    // Mineral Blocker Positions
    static final double block = 0;
    static final double dontBlock = 0.2;

    // Gold mineral X positions limits for camera.
    static final int MineralMiddleLimitLeft = 200 ;
    static final int MineralMiddleLimitRight = 400 ;


    public static final String VUFORIA_KEY = "ARCYecv/////AAABmcqxLpTXUUOPuxsa+4HIQ/GIzDMvaqWwbHZGO/Ai1kF7+COWChW41B25PqOkg6T0pwD5mJxJjStWJnIzFCHi0JyRYYqH+tscLebqWRxN7Me7udkEyQIwGw5VKxc4+gvttO/m04DvUGXEC7NjJNFtGZbbAGFBfD1UQY2vdDX1d14bIlRsHFiL9cD56NT4D0D+MACRGNnYUGs2DszENbJhIXy8uhUWeAHr3qERtEnGB0E/QoNVOxsa0G4LXl21NQhtmgYBvya9+2aC6BOjcwkEwu3XKTdYdfklbB8KNLB2+Wk6KYhTyET1YQg1+3E9asYLpnkkrZ836Y6WK7akFYds37io1yMZPRWG036tVQHfzxtD";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /* local OpMode members. */
    HardwareMap hwMap  =  null;

    /* Constructor */
    public HardwareApollo(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        //Vuforia init
        initVuforia();
        initTfod();

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize ALL Motors
        driveLeftFront  = hwMap.get(DcMotor.class, "dlf");
        driveLeftBack  = hwMap.get(DcMotor.class, "dlb");
        driveRightFront  = hwMap.get(DcMotor.class, "drf");
        driveRightBack  = hwMap.get(DcMotor.class, "drb");

        mineralGrab  = hwMap.get(DcMotor.class, "grab");
        lift  = hwMap.get(DcMotor.class, "lift");
        mineralSend  = hwMap.get(DcMotor.class, "send");

        // Define and initialize ALL servos.
        blockMineralServo  = hwMap.get(Servo.class, "block");
        mineralsDivider  = hwMap.get(Servo.class, "md");

        // Define and initialize ALL sensors
        imu = hwMap.get(BNO055IMU.class, "imu");

        // Set all motors to zero power
        setDriveMotorsPower(0, DRIVE_MOTOR_TYPES.ALL);
        mineralGrab.setPower(0);
        lift.setPower(0);
        mineralSend.setPower(0);

        driveRightBack.setDirection(DcMotor.Direction.REVERSE);     //Reverse motor
        driveRightFront.setDirection(DcMotor.Direction.REVERSE);    //Reverse motor

        // Set all servos
        mineralsDivider.setPosition(dividerMiddle);
        blockMineralServo.setPosition(block);

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
        imu.initialize(parameters);

        // Set all motors to run without encoders.
        setDriveMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralSend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    public void setDriveMotorsPosition(double Cm, DRIVE_MOTOR_TYPES driverMotorType){
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;
        double inch;
        inch = cmToInch(Cm);

        switch (driverMotorType){
            case LEFT:
                newLeftFrontTarget = driveLeftFront.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);
                newLeftBackTarget = driveLeftBack.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);

                driveLeftFront.setTargetPosition(newLeftFrontTarget);
                driveLeftBack.setTargetPosition(newLeftBackTarget);
                break;
            case RIGHT:
                newRightFrontTarget = driveRightFront.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);
                newRightBackTarget = driveRightBack.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);

                driveRightFront.setTargetPosition(newRightFrontTarget);
                driveRightBack.setTargetPosition(newRightBackTarget);
                break;
            case SIDE_WAYS:
                newLeftFrontTarget = driveLeftFront.getCurrentPosition() - (int)(inch*COUNTS_PER_INCH);
                newLeftBackTarget = driveLeftBack.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);
                newRightFrontTarget = driveRightFront.getCurrentPosition() - (int)(inch*COUNTS_PER_INCH);
                newRightBackTarget = driveRightBack.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);

                driveLeftBack.setTargetPosition(newLeftFrontTarget);
                driveLeftFront.setTargetPosition(newLeftBackTarget);
                driveRightBack.setTargetPosition(newRightFrontTarget);
                driveRightFront.setTargetPosition(newRightBackTarget);
                break;
            case ALL:
            default:
                newLeftFrontTarget = driveLeftFront.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);
                newLeftBackTarget = driveLeftBack.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);
                newRightFrontTarget = driveRightFront.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);
                newRightBackTarget = driveRightBack.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);

                driveLeftFront.setTargetPosition(newLeftFrontTarget);
                driveLeftBack.setTargetPosition(newLeftBackTarget);
                driveRightFront.setTargetPosition(newRightFrontTarget);
                driveRightBack.setTargetPosition(newRightBackTarget);
                break;
        }
    }

    //Function to set the position to all the drive motors.
    public void setLiftMotorsPosition(double ticks) {
        int newLift;
        newLift = lift.getCurrentPosition() + (int)(ticks*COUNTS_PER_INCH);
        lift.setTargetPosition(newLift);
    }

    public void setMineralSenderMotorsPosition(double ticks) {
        int newSend;
        newSend = mineralSend.getCurrentPosition() + (int)(ticks*COUNTS_PER_INCH);
        mineralSend.setTargetPosition(newSend);
    }

    //Function to set the run mode for all the drive motors.
    public void setDriveMotorsMode(DcMotor.RunMode runMode) {
        driveLeftFront.setMode(runMode);
        driveLeftBack.setMode(runMode);
        driveRightFront.setMode(runMode);
        driveRightBack.setMode(runMode);
    }

    //Function converts centimeters to inches.
    public double cmToInch(double cm){
        double inch;
        inch=cm*0.393700787;
        return inch;
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
