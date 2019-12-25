package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.round;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static org.firstinspires.ftc.teamcode.MathFunctions.dist2D;
import static org.firstinspires.ftc.teamcode.MathFunctions.getCircleLineIntersectionPoint;

public class RobotMovement extends LinearOpMode {
    Hardware robot = new Hardware();
    CurvePoint followMe;
    Orientation lastAngles = new Orientation();
    static final double P_TURN_COEFF = 0.01;     // Larger is more responsive, but also less stable
    double error;
    double steer;
    double disaierd_angle;
    double globalAngle, power = 0.70, correction;
    PIDController pidDrive;
    PIDController pidDriveEncoder;

    RobotMovement(){
        pidDrive = new PIDController(.05, 0, 0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        pidDriveEncoder = new PIDController(0.005,0,0);
        pidDriveEncoder.setOutputRange(0, power);
        pidDriveEncoder.setInputRange(-90,90);
        pidDriveEncoder.enable();

    }

    public ArrayList<CurvePoint> followCurve(ArrayList<CurvePoint> allPoints, double speed) {
        followMe = getFollowPointPath(allPoints, allPoints.get(0).followDistance);
        telemetry.addData("followMe:", "X => " + followMe.x + " Y => " + followMe.y);


        //double x, double y, double speed, double P_DRIVE_COEFF, ArrayList<CurvePoint> path, double leftSpeed, double rightSpeed, double max
        return gotoPosition(followMe.x, followMe.y, allPoints);
    }

    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, double followRadius) {

        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size() - 1; i++) {
            telemetry.addData("i", i);
            CurvePoint startLine = pathPoints.get(0);//            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(1);//             CurvePoint endLine = pathPoints.get(i + 1);
            List<Point> intersections = getCircleLineIntersectionPoint(robot.point(), followRadius, startLine.toPoint(), endLine.toPoint());
            //telemetry.addData("intersections.size()",intersections.size());
            double longestDistance = 10000000;

            for (Point thisIntersection : intersections) {
                double distance = abs(dist2D(thisIntersection, robot.point()));

                if (distance < longestDistance) {
                    telemetry.addData("thisIntersection", new Point(thisIntersection.x, thisIntersection.y));
                    longestDistance = distance;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    @Override
    public void runOpMode() {
    }

    /**
     * @param x
     * @param y
     * @param path
     * @return
     */
    public ArrayList<CurvePoint> gotoPosition(double x, double y, ArrayList<CurvePoint> path) {

        double hypotenuse = Math.hypot(x,y);
        double desiredAngle = Math.toDegrees(Math.atan2(y,x));
        pidDrive.setSetpoint(desiredAngle);
        pidDriveEncoder.setSetpoint(hypotenuse);

        correction = pidDrive.performPID(robot.GetGyroAngle());
        power = pidDriveEncoder.performPID(robot.getY());

        robot.setDriveMotorsPower(power - correction, Hardware.DRIVE_MOTOR_TYPES.LEFT);
        robot.setDriveMotorsPower(power + correction, Hardware.DRIVE_MOTOR_TYPES.RIGHT);


        for(int i = 0; i < path.size(); i++){
            if(path.get(i).toPoint().equals(new Point(robot.getX(), robot.getY()))){
                path.remove(path.get(i));
            }
        }
        return path;
    }


    /**
     *
     * @param angle
     * @param power
     */
    private void turn(double angle,double power) {
        double leftfrontSpeed;
        double rightfrontSpeed;
        double leftbackSpeed;
        double rightbackSpeed;

        double error;
        double steer;;

        error = getError(angle);
        steer = getSteer(error, P_TURN_COEFF);
        steer = limit(steer,power);
        leftfrontSpeed = robot.driveLeftFront.getPower() + steer;
        rightfrontSpeed = robot.driveRightFront.getPower() - steer;
        leftbackSpeed = robot.driveLeftBack.getPower() + steer;
        rightbackSpeed = robot.driveRightBack.getPower() - steer;


        robot.driveLeftFront.setPower(leftfrontSpeed);
        robot.driveRightFront.setPower(rightfrontSpeed);
        robot.driveLeftBack.setPower(leftbackSpeed);
        robot.driveRightBack.setPower(rightbackSpeed);
    }

    /**
     *
     * @param powerX
     * @param powerY
     */
    public void drive(double powerX, double powerY){


        double power = hypot(powerX,powerY);
        double Degree = robot.GetGyroAngle() + atan2(powerY,powerX);

        telemetry.addData("befor","X - " + powerX + "\n Y - " + powerY);
        powerY = power * sin(Degree);
        powerX = power * cos(Degree);

        telemetry.addData("after","X - " + powerX + "\n Y - " + powerY);

        robot.driveLeftBack.setPower(powerX);
        robot.driveRightFront.setPower(powerX);

        robot.driveRightBack.setPower(powerY);
        robot.driveLeftFront.setPower(powerY);
    }


    /**
     *
     * @param targetAngle
     * @return
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
        return error * PCoeff;
    }
    

    public double botX(){
        return robot.getX() + 283.5;
    }

    public double botY(){
        return abs(robot.getY() - 283.5);
    }

    /**
     *
     * @param value
     * @param maxmin
     * @return
     */
    public double limit(double value,double maxmin) {
        return max(-maxmin, min(value, maxmin));
    }

    private double getAngle(){
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }





}

