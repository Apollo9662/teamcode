package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

public class GyroOperator extends LinearOpMode {
    static final double HEADING_THRESHOLD = 5;      // As tight as e can make it with an integer gyro
    static final double P_DRIVE_COEFF = 0.01;     // Larger is more responsive, but also less stable
    static final double P_TURN_COEFF = 0.05;     // Larger is more responsive, but also less stable

    static final double COUNTS_PER_MOTOR_REV = 280;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    enum DriveMode {
        normal,
        left,
        right;
    }

    // liot test vofuria eithout robot
    Hardware robot = new Hardware();


    @Override
    public void runOpMode() {
    }

    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setDriveMotorsPower(leftSpeed, Hardware.DRIVE_MOTOR_TYPES.LEFT);
        robot.setDriveMotorsPower(rightSpeed, Hardware.DRIVE_MOTOR_TYPES.RIGHT);


        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.GetGyroAngle();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public void pidTurn(double speed,double angle){
        PIDController pid = new PIDController(0.02,0,0);
        pid.setSetpoint(angle);
        pid.setOutputRange(0, speed);
        pid.setInputRange(-180, 180);
        pid.enable();
        double power;
        double error = angle - robot.GetGyroAngle();
        while(abs(error) > 2){
            power = pid.performPID(robot.GetGyroAngle());
            robot.setDriveMotorsPower(-power, Hardware.DRIVE_MOTOR_TYPES.LEFT);
            robot.setDriveMotorsPower(power, Hardware.DRIVE_MOTOR_TYPES.RIGHT);
            error = angle - robot.GetGyroAngle();
            telemetry.addData("eror", error);
            telemetry.update();
        }
        robot.setDriveMotorsPower(0, Hardware.DRIVE_MOTOR_TYPES.ALL);

            telemetry.addData("eror", error);
            telemetry.update();

        Log.d("YAIR","1");
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void pidDrive(double speed, double distance,double angle,DriveMode mode){
        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double steer;
        double leftSpeed;
        double rightSpeed;

        PIDController pidDrive;
        PIDController[] pidDriveEncoder = new PIDController[2];

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            this.robot.driveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.robot.driveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            this.robot.driveRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.robot.driveLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            boolean leftOnTarget = false;
            boolean rightOnTarget = false;
            pidDrive = new PIDController(.025, 0, 0);

            pidDriveEncoder[0] = new PIDController(0.005,0,0);
            pidDriveEncoder[1] = new PIDController(0.005,0,0);

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = this.robot.driveLeftBack.getCurrentPosition() + moveCounts;
            newRightTarget = this.robot.driveRightBack.getCurrentPosition() + moveCounts;

            pidDrive.setSetpoint(angle);
            pidDrive.setOutputRange(0, speed);
            pidDrive.setInputRange(-180, 180);
            pidDrive.enable();
            Log.d("YAIR","1");


            pidDriveEncoder[0].setSetpoint(newLeftTarget);
            pidDriveEncoder[0].setOutputRange(-speed, speed);
            pidDriveEncoder[0].setInputRange(-3000,3000);
            pidDriveEncoder[0].enable();
            Log.d("YAIR","2");

            pidDriveEncoder[1] = pidDriveEncoder[0];
            pidDriveEncoder[1].setSetpoint(newRightTarget);
            Log.d("YAIR","3");



            //set targets
            this.robot.driveLeftBack.setTargetPosition(newLeftTarget);
            this.robot.driveRightBack.setTargetPosition(newRightTarget);
            Log.d("YAIR","4");


            if (motorTarget(this.robot.driveLeftBack) || motorTarget(this.robot.driveRightBack)){
                if (opModeIsActive()){
                    Log.d("YAIR","13");
                }
            }
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && !rightOnTarget && !leftOnTarget) {
                rightOnTarget = (motorTarget(this.robot.driveRightBack));
                leftOnTarget = (motorTarget(this.robot.driveLeftBack));
                Log.d("YAIR","5");


                steer = pidDrive.performPID(robot.GetGyroAngle());
                leftSpeed = pidDriveEncoder[0].performPID(robot.driveLeftBack.getCurrentPosition());
                rightSpeed = pidDriveEncoder[1].performPID(robot.driveRightBack.getCurrentPosition());
                leftSpeed  -= steer;
                rightSpeed += steer;
                Log.d("YAIR","6");


                switch (mode) {
                    case left:
                        robot.driveLeftFront.setPower(-rightSpeed);
                        robot.driveLeftBack.setPower(leftSpeed);

                        robot.driveRightFront.setPower(rightSpeed);
                        robot.driveRightBack.setPower(-leftSpeed);
                        telemetry.addData("mode",mode);
                        break;
                    case right:
                        robot.driveLeftFront.setPower(leftSpeed);
                        robot.driveLeftBack.setPower(-rightSpeed);

                        robot.driveRightFront.setPower(-leftSpeed);
                        robot.driveRightBack.setPower(rightSpeed);
                        break;

                    default:
                        this.robot.setDriveMotorsPower(leftSpeed, Hardware.DRIVE_MOTOR_TYPES.LEFT);
                        this.robot.setDriveMotorsPower(rightSpeed, Hardware.DRIVE_MOTOR_TYPES.RIGHT);
                        break;

                }
                Log.d("YAIR","7");

                // Display drive status for the driver.
                telemetry.addData("Err/St", steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", this.robot.driveLeftBack.getCurrentPosition(),
                        this.robot.driveRightBack.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.addData("leftOnTarget =>", leftOnTarget);
                telemetry.addData("rightOnTarget =>", rightOnTarget);
                telemetry.update();
            }

            // Stop all motion;
            Log.d("YAIR","8");

            this.robot.setDriveMotorsPower(0, Hardware.DRIVE_MOTOR_TYPES.ALL);
            pidDriveEncoder[0].disable();
            pidDriveEncoder[0].disable();

        }
        Log.d("YAIR","9");
    }

    public void PIDTurn(double speed, double angle){

    }


    public void drive(double speed,
                      double distance,
                      double angle, DriveMode mode) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        boolean leftOnTarget = false;
        boolean rightOnTarget = false;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH) * (int)(speed/abs(speed));
            newLeftTarget = this.robot.driveLeftBack.getCurrentPosition() + moveCounts;
            newRightTarget = this.robot.driveRightBack.getCurrentPosition() + moveCounts;

            //set targets
            this.robot.driveLeftBack.setTargetPosition(newLeftTarget);
            this.robot.driveRightBack.setTargetPosition(newRightTarget);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && !rightOnTarget && !leftOnTarget) {

                rightOnTarget = (motorTarget(this.robot.driveRightBack));
                leftOnTarget = (motorTarget(this.robot.driveLeftBack));
                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(abs(leftSpeed), abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
                switch (mode) {
                    case left:
                        robot.driveLeftFront.setVelocity(-rightSpeed);
                        robot.driveLeftBack.setVelocity(leftSpeed);

                        robot.driveRightFront.setVelocity(rightSpeed);
                        robot.driveRightBack.setVelocity(-leftSpeed);
                        break;
                    case right:
                        robot.driveLeftFront.setVelocity(leftSpeed);
                        robot.driveLeftBack.setVelocity(-rightSpeed);

                        robot.driveRightFront.setVelocity(-leftSpeed);
                        robot.driveRightBack.setVelocity(rightSpeed);
                        break;

                    default:
                        this.robot.setDriveMotorsPower(leftSpeed, Hardware.DRIVE_MOTOR_TYPES.LEFT);
                        this.robot.setDriveMotorsPower(rightSpeed, Hardware.DRIVE_MOTOR_TYPES.RIGHT);
                        break;


                }
                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", this.robot.driveLeftBack.getCurrentPosition(),
                        this.robot.driveRightBack.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.addData("leftOnTarget =>", leftOnTarget);
                telemetry.addData("rightOnTarget =>", rightOnTarget);
                telemetry.update();
            }

            // Stop all motion;
            this.robot.setDriveMotorsPower(0, Hardware.DRIVE_MOTOR_TYPES.ALL);
        }
    }

    public boolean motorTarget(DcMotor Motor) {
            if (abs(Motor.getCurrentPosition()) < abs(Motor.getTargetPosition())) {
                return false;
            } else {
                return true;
            }

    }
}


