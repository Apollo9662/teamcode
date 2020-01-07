package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.AutoRed.stonePositionNewMoves.*;
import static org.firstinspires.ftc.teamcode.GyroOperator.DriveMode.left;
import static org.firstinspires.ftc.teamcode.GyroOperator.DriveMode.normal;
import static org.firstinspires.ftc.teamcode.GyroOperator.DriveMode.right;

@Autonomous(name = "Autonomous blue", group = "apollo")
public class AutoRed extends autonum{
    enum stonePositionNewMoves{
        LEFT,
        CENTER,
        RIGHT
    }
    private vuforia vuforia = new vuforia();
    public void runOpMode(){
        robot.init(hardwareMap, false);
        vuforia.init(hardwareMap);
        telemetry.addData("state ==> ", "Start");
        telemetry.update();

        waitForStart();
       // robot.setCatchers(1);
        //newRight();
        stonePositionNewMoves vu = vufria();
        telemetry.addData("pos  - ",vu);
        telemetry.update();
        sleep(700);
        switch (vu){
            case RIGHT:
                newRight();
                break;
            case CENTER:
                newCenter();
                break;
            case LEFT:
                newLeft();
                break;
        }
        pidDrive(0.2,-18,0,normal);
        robot.setCatchers(0);
        robot.horizontalElevator.setPower(-1);
        robot.verticalElevator.setPower(0.2);
        sleep(1800);
        robot.backClaw.setPosition(0);
        robot.verticalElevator.setPower(0);
        robot.horizontalElevator.setPower(0);
        sleep(300);
//        robot.horizontalElevator.setPower(1);
//        sleep(700);
//        robot.horizontalElevator.setPower(0);
        pidDrive(0.55,90,10,normal);
        robot.setCatchers(1);
        pidTurn(0.5,90);
        sleep(800);
        pidDrive(0.4,70,90,normal);

}
    public stonePositionNewMoves vufria(){
        pidDrive(0.3,40,0,normal);
        pidDrive(0.3,16,0,right);
        if (vuforia.procces() != 10000) {
            return LEFT;
        }
        pidDrive(0.3,17 ,0,right);
        sleep(400);
        if (vuforia.procces() != 10000) {
            return CENTER;
        }
        return RIGHT;

    }
    public void newRight(){
        robot.backClaw.setPosition(1);
        robot.verticalElevator.setPower(1);
        sleep(350);
        robot.horizontalElevator.setPower(0);
        robot.verticalElevator.setPower(0);
        robot.frontClaw.setPosition(0.07);
        robot.backClaw.setPosition(1);
        pidTurn(0.4,-90);
        pidDrive(0.4,-7,-90,normal);
        pidDrive(0.35,60,-90,left);
        robot.setCollectMotorsPower(1);
        pidDrive(0.4,17,-90,normal);
        sleep(600);
        robot.frontClaw.setPosition(0.85);
        robot.setCollectMotorsPower(0);
        robot.setCollectMotorsPower(0);
        robot.setCollectMotorsPower(-0.4);
        pidDrive(0.5,67,-90,right);
        robot.setCollectMotorsPower(0);
        pidDrive(0.6,-150,-90,normal);
        pidTurn(0.2,-180);
        robot.initImu();

    }
    public void newCenter(){
        robot.backClaw.setPosition(1);
        robot.verticalElevator.setPower(1);
        sleep(350);
        robot.horizontalElevator.setPower(0);
        robot.verticalElevator.setPower(0);
        robot.frontClaw.setPosition(0.07);
        robot.backClaw.setPosition(1);
        pidTurn(0.4,-90);
        pidDrive(0.4,23,-90,normal);
        pidDrive(0.35,60,-90,left);
        robot.setCollectMotorsPower(1);
        pidDrive(0.4,10,-90,normal);
        sleep(600);
        robot.frontClaw.setPosition(0.85);
        robot.setCollectMotorsPower(0);
        robot.setCollectMotorsPower(-0.4);
        pidDrive(0.5,67,-90,right);
        robot.setCollectMotorsPower(0);
        pidDrive(0.6,-175,-90,normal);
        pidTurn(0.2,-180);
        robot.initImu();

    }
    public void newLeft(){
        robot.backClaw.setPosition(1);
        robot.verticalElevator.setPower(1);
        sleep(350);
        robot.horizontalElevator.setPower(0);
        robot.verticalElevator.setPower(0);
        robot.frontClaw.setPosition(0.07);
        robot.backClaw.setPosition(1);
        pidTurn(0.4,-90);
        pidDrive(0.4,17,-90,normal);
        pidDrive(0.35,60,-90,left);
        robot.setCollectMotorsPower(1);
        pidDrive(0.4,17,-90,normal);
        sleep(600);
        robot.frontClaw.setPosition(0.85);
        robot.setCollectMotorsPower(0);
        robot.setCollectMotorsPower(-0.4);
        pidDrive(0.5,67,-90,right);
        robot.setCollectMotorsPower(0);
        pidDrive(0.6,-163,-90,normal);
        pidTurn(0.2,-170);
        robot.initImu();

    }
}
