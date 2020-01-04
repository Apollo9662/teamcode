
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    private DcMotor rightCollector = null;
    private DcMotor leftCollector = null;

    Servo frontClaw = null;
    Servo backClaw = null;

    private Servo rightCatcher = null;
    private Servo leftCatcher = null;

    DcMotor verticalElevator = null;
    DcMotor horizontalElevator = null;

    private int encodersTicks = 800;
    private double pulleyCircumference = 2 * Math.PI;
    private double ticksPerInch = 800/pulleyCircumference;

    BNO055IMU imu;

    public Point point() {
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
        /* local OpMode members. */
        imu = ahwMap.get(BNO055IMU.class, "imu");
        initImu();
        // Save reference to Hardware map


        // Define and Initialize Motors
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

        frontClaw = ahwMap.get(Servo.class,"os2");
        backClaw = ahwMap.get(Servo.class,"os" );


        driveRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRightFront.setDirection(DcMotorSimple.Direction.REVERSE);



        verticalElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        driveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        driveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(!teleop) {
            driveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }else{
            driveRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

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
            leftCatcher.setPosition(0.45);
        }

    }
    public double getX(double angle){

        return driveLeftBack.getCurrentPosition() - reset.x;
    }

    public double getY(){
        return driveRightBack.getCurrentPosition() - reset.y;
    }

    public int calculateXY(double angle, int lastCurrentPositionX, int lastCurrentPositionY){
        if(!(Math.abs(this.XEncoder() - lastCurrentPositionX) < (int)(ticksPerInch)) || !(Math.abs(this.YEncoder() - lastCurrentPositionY) < (int)(ticksPerInch))) {
            if (angle > 45 && angle <= 135) {

            }
            else if (angle > 135 && angle <= 225) {
                position.x -= Math.cos(angle) * this.XEncoder() - lastCurrentPositionX;
                position.y -= Math.sin(angle) * this.YEncoder() - lastCurrentPositionY;
            }
            else if(angle > 225 && angle <= 315){

            }
            else {
                position.x += Math.cos(angle) * this.XEncoder() - lastCurrentPositionX;
                position.y += Math.sin(angle) * this.YEncoder() - lastCurrentPositionY;
            }
        }
    }

    public void initImu(){
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
        imu.initialize(parameters);
    }

    public int XEncoder(){
        return this.rightCollector.getCurrentPosition();
    }

    public int YEncoder(){
        return this.leftCollector.getCurrentPosition();
    }
 }


