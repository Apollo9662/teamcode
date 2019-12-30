package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.GyroOperator.DriveMode.*;


@Autonomous(name = "auto1")
public abstract class autonum extends GyroOperator {
    int counter = 0;
    double stonePosition;
    int counterRight = 0;
    int counterLeft = 0;
    int counterCenter = 0;

    @Override
    public void runOpMode() {
    }


    public void pathRight(int red){
        robot.backClaw.setPosition(1);
        robot.verticalElevator.setPower(1);
        sleep(350);
        robot.horizontalElevator.setPower(0);
        robot.verticalElevator.setPower(0);
        robot.frontClaw.setPosition(1);
        robot.setCollectMotorsPower(1);
        pidDrive(0.4, 110, red*-43, normal);
        robot.horizontalElevator.setPower(0);
        robot.verticalElevator.setPower(0.1);
        sleep(1300);
        robot.verticalElevator.setPower(0);
        robot.frontClaw.setPosition(0);
        pidDrive(0.4, -28, red*-43, normal);
        sleep(300);
        pidDrive(0.5, -163, red*-90, normal);
        robot.setCollectMotorsPower(0);
        Log.d("YAIR","11");
        pidDrive(0.4, -65   , red*-180, normal);
        robot.initImu();
        robot.horizontalElevator.setPower(-1);
        sleep(1550);
        robot.backClaw.setPosition(0);
        robot.horizontalElevator.setPower(0);
        sleep(300);
        robot.horizontalElevator.setPower(1);
        sleep(700);
        robot.horizontalElevator.setPower(0);
        robot.setCatchers(0);
        sleep(800);
        pidDrive(0.4, 55, red*90, normal);
        pidDrive(0.4, -45, red*90, normal);

    }

    public void pathCenter(int red){
        robot.backClaw.setPosition(1);
        robot.verticalElevator.setPower(1);
        sleep(350);
        robot.horizontalElevator.setPower(0);
        robot.verticalElevator.setPower(0);
        robot.setCollectMotorsPower(0.7);
        pidDrive(0.3, 98, red*-31, normal);
        robot.horizontalElevator.setPower(0);
        robot.verticalElevator.setPower(0.1);
        sleep(1500);
        robot.verticalElevator.setPower(0);
        robot.frontClaw.setPosition(0);
        pidDrive(0.4, -23, red*-33, normal);
        sleep(300);
        pidDrive(0.5, -170, -90, normal);
        robot.setCollectMotorsPower(0);
        Log.d("YAIR","11");
        pidDrive(0.4, -65   , -180, normal);
        robot.initImu();
        robot.horizontalElevator.setPower(-1);
        sleep(1550);
        robot.backClaw.setPosition(0);
        robot.horizontalElevator.setPower(0);
        sleep(300);
        robot.horizontalElevator.setPower(1);
        sleep(700);
        robot.horizontalElevator.setPower(0);

    }
    public void pathLeft(int red){
        robot.backClaw.setPosition(1);
        robot.verticalElevator.setPower(1);
        sleep(350);
        robot.horizontalElevator.setPower(0);
        robot.verticalElevator.setPower(0);
        robot.setCollectMotorsPower(0.8);
        robot.frontClaw.setPosition(1);
        pidDrive(0.4, 87, red*-25, normal);
        robot.horizontalElevator.setPower(0);
        robot.verticalElevator.setPower(0.1);
        sleep(1300);
        robot.verticalElevator.setPower(0);
        robot.frontClaw.setPosition(0);
        pidDrive(0.4, -20, red*-25, normal);
        sleep(300);
        pidDrive(0.6, -162, red*-90, normal);
        robot.setCollectMotorsPower(0);
        Log.d("YAIR","11");
        pidDrive(0.4, -65   , red*-180, normal);
        robot.initImu();
        robot.horizontalElevator.setPower(-1);
        sleep(1550);
        robot.backClaw.setPosition(0);
        robot.horizontalElevator.setPower(0);
        sleep(300);
        robot.horizontalElevator.setPower(1);
        sleep(700);
        robot.horizontalElevator.setPower(0);
    }
}