package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.MathFunctions.dist2D;
import static org.firstinspires.ftc.teamcode.MathFunctions.getCircleLineIntersectionPoint;

public class RobotMovement extends LinearOpMode {
    Hardware robot = new Hardware();
    static final double     P_TURN_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    private double mindistanceToTarget = 20;

    public ArrayList<CurvePoint> followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        CurvePoint followMe = getFollowPointPath(allPoints,allPoints.get(0).followDistance);
        telemetry.addData("followMe:","X => " + followMe.x + " Y => " + followMe.y );

        return gotoPosition(followMe.x,followMe.y,followMe.moveSpeed,followAngle,followMe.turnSpeed,allPoints);
    }

    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints,double followRadius){

        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i = 0; i < pathPoints.size() - 1; i++){
            telemetry.addData("i",i);
            CurvePoint startLine = pathPoints.get(0);//            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(1);//             CurvePoint endLine = pathPoints.get(i + 1);
            List<Point> intersections = getCircleLineIntersectionPoint(robot.position,followRadius,startLine.toPoint(),endLine.toPoint());
            //telemetry.addData("intersections.size()",intersections.size());
            double longestDistance = 10000000;

            for(Point thisIntersection : intersections){
                double distance = abs(dist2D(thisIntersection,robot.position));

                if(distance < longestDistance){
                    telemetry.addData("thisIntersection",new Point(thisIntersection.x,thisIntersection.y));
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
     *  @param x
     * @param y
     * @param movementSpeed
     * @param preferredAngle
     * @param turnSpeed
     * @return
     */
    public ArrayList<CurvePoint> gotoPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed , ArrayList<CurvePoint> path){

        double distanceToTarget = hypot(x - botX(),y - botY());
        telemetry.addData("distanceToTarget",distanceToTarget);
        double absoluteAngleToTarget = atan2(y - botY(), x - botX());
        // telemetry.addData("absoluteAngleToTarget", absoluteAngleToTarget);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (robot.GetGyroAngle() - toRadians(90)));
        // telemetry.addData("relativeAngleToPoint", relativeAngleToPoint);
        double relativeXToPoint = sin(relativeAngleToPoint) * distanceToTarget;
        //telemetry.addData("relativeXToPoint", relativeXToPoint);
        double relativeYToPoint = cos(relativeAngleToPoint) * distanceToTarget;
        //telemetry.addData("relativeYToPoint", relativeYToPoint);


        double movementXPower = relativeYToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
        //telemetry.addData("movementXPower", movementXPower);
        double movementYPower = relativeXToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
        //telemetry.addData("movementYPower", movementYPower);

        movementXPower *= movementSpeed;
        movementYPower *= movementSpeed;
        drive(movementXPower, movementYPower);
        double relativeTurnAngle = relativeAngleToPoint - toRadians(180) + preferredAngle;
        turn(relativeTurnAngle, Range.clip(relativeTurnAngle / toRadians(30), -1, 1) * turnSpeed);
        if(distanceToTarget < mindistanceToTarget){
            for(int i = 0; i < path.size();i++){
                if(path.get(i).toPoint().equals(new Point(x, y))){
                    path.remove(path.get(i));
                }
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

}
