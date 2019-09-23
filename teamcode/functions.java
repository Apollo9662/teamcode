package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "ToPointTest",group = "Apollo")
public abstract class functions extends LinearOpMode {
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable


    Hardware robot = new Hardware();
    @Override
    public void runOpMode()  {
    }
    public void driveToPosition(double targetX, double targetY){
        double distanceToTarget = Math.sqrt(Math.pow(targetX - robot.position.x,2) + Math.pow(targetY - robot.position.y,2));
        double encoder = 0;
        double distance = 0;
        while ( distanceToTarget > 20 && opModeIsActive()){
            distanceToTarget = Math.sqrt(Math.pow(targetX - robot.position.x,2) + Math.pow(targetY - robot.position.y,2));
            robot.setDriveMotorsPower(0.2, Hardware.DRIVE_MOTOR_TYPES.ALL);
            distance = robot.driveRightBack.getCurrentPosition() - encoder;
            robot.position.x += distance*Math.cos(Math.toRadians(robot.GetGyroAngle()));
            robot.position.y += distance*Math.sin(Math.toRadians(robot.GetGyroAngle()));
            telemetry.addData("state","driving");
            telemetry.addData("robot.position.x",robot.position.x);
            telemetry.addData("robot.position.y",robot.position.y);
            telemetry.addData("dic",distance);
            telemetry.addData("dic to target",distanceToTarget);
            telemetry.addData("encoer",robot.driveRightBack.getCurrentPosition());
            telemetry.update();

            encoder = robot.driveRightBack.getCurrentPosition();
        }




    }
    public void turn(double speed, double dgreeTarget){
        double motorPower = 0;
        while (robot.GetGyroAngle() == dgreeTarget) {
            motorPower = (dgreeTarget - robot.GetGyroAngle()) / 180;
            robot.setDriveMotorsPower(motorPower * (speed + 1), Hardware.DRIVE_MOTOR_TYPES.RIGHT);
            robot.setDriveMotorsPower(-motorPower * (speed + 1), Hardware.DRIVE_MOTOR_TYPES.LEFT);
        }



    }
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
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
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.GetGyroAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}


