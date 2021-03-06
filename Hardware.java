
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware {
    Point position = new Point(0,0);
    /* Public OpMode members. */
    private Point reset = new Point(0,0);

    DcMotorEx driveLeftBack = null;
    DcMotorEx driveRightBack = null;
    DcMotorEx driveLeftFront = null;
    DcMotorEx driveRightFront = null;

    DcMotor rightCollector = null;
    DcMotor leftCollector = null;

    Servo frontClaw = null;
    Servo backClaw = null;

    private Servo rightCatcher = null;
    private Servo leftCatcher = null;


    DcMotor verticalElevator = null;
    DcMotor horizontalElevator = null;


    BNO055IMU imu;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;


    public static double frontClosePos = 0.9;
    public static double frontOpenPos = 0.07;

    public static double backClosePos =1;
    public static double  backOpenPos = 0.2;
    Point point() {
        return new Point(getX(),getY());
    }

    void reverse() {
        driveLeftFront.setDirection(driveLeftFront.getDirection().inverted());
        driveRightFront.setDirection(driveRightFront.getDirection().inverted());
        driveRightBack.setDirection(driveRightBack.getDirection().inverted());
        driveLeftBack.setDirection(driveLeftBack.getDirection().inverted());
    }


    public enum DRIVE_MOTOR_TYPES {
        LEFT,
        RIGHT,
        FRONT,
        BACK,
        SIDE_WAYS,
        DIAGONAL_RIGHT,
        DIAGONAL_LEFT,
        ALL
    }

    /* Constructor */
    public Hardware(){

    }
    float GetGyroAngle(){

        Orientation angles =imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
    }

    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap,boolean teleop) {
        boolean sucses;
        int counter = 0;
        /* local OpMode members. */
        imu = ahwMap.get(BNO055IMU.class, "imu ");
        if (initImu()){}
        else {
            imu = ahwMap.get(BNO055IMU.class , " imu 1");
            initImu();
        }
        //do {
        //    sucses = initImu();
         //   counter++;

        //} while (!sucses && counter < 7);
        // Save reference to Hardware map


        sensorColor =ahwMap.get(ColorSensor.class, "scd");

        sensorDistance = ahwMap.get(DistanceSensor.class, "scd");

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = ahwMap.appContext.getResources().getIdentifier("RelativeLayout", "id", ahwMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) ahwMap.appContext).findViewById(relativeLayoutId);


        driveLeftBack = ahwMap.get(DcMotorEx.class, "dlb");
        driveRightBack = ahwMap.get(DcMotorEx.class, "drb");
        driveRightFront = ahwMap.get(DcMotorEx.class, "drf");
        driveLeftFront = ahwMap.get(DcMotorEx.class, "dlf");

        rightCollector = ahwMap.get(DcMotor.class, "rc");
        leftCollector = ahwMap.get(DcMotor.class, "lc");

        verticalElevator = ahwMap.get(DcMotor.class, "eu");
        horizontalElevator = ahwMap.get(DcMotor.class, "es");

        rightCatcher = ahwMap.get(Servo.class, "rca");
        leftCatcher = ahwMap.get(Servo.class, "lca");

        frontClaw = ahwMap.get(Servo.class,"os");
        backClaw = ahwMap.get(Servo.class,"os2" );


        driveRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRightFront.setDirection(DcMotorSimple.Direction.REVERSE);



        verticalElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        driveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        driveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(!teleop) {
            driveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            driveRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        rightCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        reset.x = getX();
        reset.y = getY();

    }


    void setDriveMotorsPower(double power, DRIVE_MOTOR_TYPES driverMotorType){
        switch (driverMotorType){
            case LEFT:

                driveLeftFront.setPower(power);
                driveLeftBack.setPower(power);
                break;
            case RIGHT:

                driveRightBack.setPower(power);
                driveRightFront.setPower(power);
                break;
            case SIDE_WAYS:

                driveRightBack.setPower(power);
                driveRightFront.setPower(-power);
                driveLeftFront.setPower(power);
                driveLeftBack.setPower(-power);
                break;
            case DIAGONAL_LEFT:
                driveRightFront.setPower(power);
                driveLeftBack.setPower(power);
                break;
            case DIAGONAL_RIGHT:

               driveLeftFront.setPower(power);
               driveRightBack.setPower(power);

                break;
            case BACK:
                driveLeftBack.setPower(power);
                driveRightBack.setPower(power);
                break;

            case FRONT:
                driveLeftFront.setPower(power);
                driveRightFront.setPower(power);
                break;
            case ALL:
            default:
             //  driveLeftFront.setVelocity(power);
             //  driveLeftBack.setVelocity(power);
             //  driveRightFront.setVelocity(power);
               driveRightBack.setVelocity(power);

                driveLeftFront.setPower(power);
                driveLeftBack.setPower(power);
                driveRightBack.setPower(power);
                driveRightFront.setPower(power);
                break;
        }
    }

    void setCollectMotorsPower(double power){
        rightCollector.setPower(power);
        leftCollector.setPower(-power);
    }

    void setCatchers(double position){
        if(position == 1){
            rightCatcher.setPosition(0.45);
            leftCatcher.setPosition(0);
        }else{
            rightCatcher.setPosition(0);
            leftCatcher.setPosition(0.55);
        }

    }
    public double getX(){
        return driveLeftBack.getCurrentPosition() - reset.x;
    }

    public double getY(){
        return driveRightBack.getCurrentPosition() - reset.y;
    }
    public boolean initImu(){
        boolean sucses;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        sucses = imu.initialize(parameters);
        return sucses;


    }
 }


