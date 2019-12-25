package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.GyroOperator.DriveMode.*;


@Autonomous(name = "auto1")
public class autonum extends GyroOperator {
    private ConceptVuforiaSkyStoneNavigation vuforia = new ConceptVuforiaSkyStoneNavigation();
    int counter = 0;
    ConceptVuforiaSkyStoneNavigation.stonePositionEnum stonePosition;
    ConceptVuforiaSkyStoneNavigation.stonePositionEnum lastStonePosition = ConceptVuforiaSkyStoneNavigation.stonePositionEnum.CENTER;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap,false);
        // vuforia.vuforiaInit(hardwareMap);
        telemetry.addData("state ==> ", "Start");
        telemetry.update();
        waitForStart();
//        vuforia.enable(true);

        if (opModeIsActive()) {
            robot.frontClaw.setPosition(1);
            pathRight();


            /*while (counter < 5) {
//                stonePosition = vuforia.process();
//                if (vuforia.targetVisible) {
//                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                            vuforia.translation.get(0) / vuforia.mmPerInch, vuforia.translation.get(1) / vuforia.mmPerInch,
//                            vuforia.translation.get(2) / vuforia.mmPerInch);
//                }
//                if (stonePosition == lastStonePosition) {
//                    counter++;
//                } else {
//                    counter = 1;
//                }
//                lastStonePosition = stonePosition;
//
//                Log.d("counter", "value: " + counter);
//                telemetry.update();
//            }
*/
        }

    }


    public void pathRight() {
        robot.backClaw.setPosition(1);
        robot.verticalElevator.setPower(1);
        sleep(350);
        robot.horizontalElevator.setPosition(0);
        robot.verticalElevator.setPower(0);
        robot.setCollectMotorsPower(1);
        pidDrive(0.4, 110, -43, normal);
        robot.horizontalElevator.setPosition(0);
        robot.verticalElevator.setPower(0.1);
        sleep(1300);
        robot.verticalElevator.setPower(0);
        robot.frontClaw.setPosition(0);
        pidDrive(0.4, -7, -43, normal);
        pidDrive(0.4, -15, -43, normal);
        robot.horizontalElevator.setPosition(0.8);
        sleep(300);
        pidDrive(0.5, -148, -90, normal);
        robot.setCollectMotorsPower(0);
        Log.d("YAIR","11");
        pidDrive(0.4, -60, -180, normal);
        robot.initImu();

        sleep(1000);
        robot.backClaw.setPosition(0);
        robot.horizontalElevator.setPosition(0);
        sleep(300);
        robot.horizontalElevator.setPosition(0.2);
        sleep(500);
        pidDrive(0.4, 45, 90, normal);
        pidDrive(0.5, 116, 90, normal);
        robot.initImu();
        pidDrive(0.5, 20, 30, normal);

    }

    public void pathCenter(){
        robot.verticalElevator.setPower(1);
        robot.verticalElevator.setPower(0);
        robot.setCollectMotorsPower(0.8);
        pidDrive(0.3, 80, -30, normal);
        sleep(300);
        robot.frontClaw.setPosition(0);
        pidDrive(0.3, -40, -30, normal);
        Log.d("YAIR","10");
        robot.horizontalElevator.setPosition(0.8);
        pidDrive(0.4, -140, -90, normal);
        robot.setCollectMotorsPower(0);
        Log.d("YAIR","11");
//        pidTurn(-0.4,180);
        pidDrive(0.3, -80, -180, normal);
        robot.initImu();

        sleep(5000);
        robot.horizontalElevator.setPosition(0);
        robot.frontClaw.setPosition(0);
        robot.backClaw.setPosition(0.8);

    }
}