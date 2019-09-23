package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
public class Auto extends LinearOpMode {
    RobotMovement robotMovement = new RobotMovement();
    ElapsedTime runtime = new ElapsedTime();
    ArrayList<CurvePoint> pathCurrent = new ArrayList();
    ArrayList<CurvePoint> path = new ArrayList();
    public void runOpMode(){
        robotMovement.robot.init(hardwareMap);
//        path.add(new CurvePoint(robotMovement.botX(),robotMovement.botY(),1.0,0.5,50,Math.toRadians(0),1.0));
        path.add(new CurvePoint(100,100,1.0,0.5,50,Math.toRadians(30),1.0));
        path.add(new CurvePoint(300,100,1.0,0.5,50,Math.toRadians(30),1.0));
        path.add(new CurvePoint(150,0,1.0,0.5,50,Math.toRadians(30),1.0));
        telemetry.addData("finish","wating to start");
        telemetry.update();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            telemetry.addData("opMose","is active");
            telemetry.addData("bot point", new Point(robotMovement.botX(), robotMovement.botY()));
            if(pathCurrent.size() > 0) {
                pathCurrent = robotMovement.followCurve(pathCurrent, Math.toRadians(30));
            }
            else{
                robotMovement.drive(0, 0);
                runtime.reset();
                if(path.size() > 0){
                    runtime.reset();
                    pathCurrent.add(path.get(0));
                    path.remove(path.get(0));
                    runtime.reset();
                }
            }
            telemetry.update();

        }
    }
}
