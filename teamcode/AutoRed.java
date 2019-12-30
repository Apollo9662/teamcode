package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.GyroOperator.DriveMode.left;
import static org.firstinspires.ftc.teamcode.GyroOperator.DriveMode.normal;
import static org.firstinspires.ftc.teamcode.GyroOperator.DriveMode.right;

@Autonomous(name = "Autonomous red", group = "apollo")
public class AutoRed extends autonum{
    private vuforia vuforia = new vuforia();
    public void runOpMode(){
        robot.init(hardwareMap, false);
        telemetry.addData("state ==> ", "Start");
        telemetry.update();
        waitForStart();
        robot.setCatchers(1);
        newRight();
//        if (opModeIsActive()) {
//
//        //          pidDrive(0.3,85,90,normal);
//        pidDrive(0.2, 14, 0, normal);
//        sleep(1000);
//
//        double y = vuforia.procces(hardwareMap);
//        if (y > 0 && y != 10000 ){
//            //  pathRight(1);
//        } else if (y<0 ){
//            //  pathCenter(1);
//        } else {
//            // pathLeft(1);
//        }
//        while (opModeIsActive()) {
//            telemetry.addData("hosfd", y);
//
//            telemetry.update();
////
////            }
//            //  robot.frontClaw.setPosition(1);
//            //  if (stonePosition == RIGHT){
//            //      pathRight();
//            //  }
//            //  if (stonePosition == LEFT){
//            //      pathLeft();
//            //  }
//            //  if (stonePosition == CENTER){
//            //      pathCenter();
//            //  }
//            //
//            //  //pathRight();
////        //    pathLeft();
//            //  robot.setCatchers(1);
//            //  pidDrive(0.4,50,90,normal);
//            //  pidDrive(0.4,-30,90,normal);
//
//        }
//
//
//        //       while (counter < 5) {
//        //       stonePosition = vuforia.process();
//        //          if (vuforia.targetVisible) {
//        //              telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//        //                      vuforia.translation.get(0) / vuforia.mmPerInch, vuforia.translation.get(1) / vuforia.mmPerInch,
//        //                      vuforia.translation.get(2) / vuforia.mmPerInch);
//        //          }
//        //          if (stonePosition == lastStonePosition) {
//        //              counter++;
//        //          } else {
//        //              counter = 1;
//        //          }
//        //          lastStonePosition = stonePosition;
//
//        //          Log.d("counter", "value: " + counter);
//        //          telemetry.update();
//        //      }
//
//        //   }
//
//    }
}
    public void newRight(){
        robot.frontClaw.setPosition(1);
        robot.backClaw.setPosition(1);
        pidDrive(0.4,37,0,normal);
        pidTurn(0.3,-90);
        pidDrive(0.3,17,-90,normal);
        pidDrive(0.4,70,-90,left);
        robot.setCollectMotorsPower(1);
        pidDrive(0.4,15,-90,normal);
        sleep(600);
        robot.frontClaw.setPosition(0);
        pidDrive(0.4,70,-90,right);
        pidDrive(0.5,-160,-90,normal);
        pidTurn(0.2,-180);
        robot.initImu();
        pidDrive(0.3,-15,0,normal);
        robot.setCatchers(0);
        robot.horizontalElevator.setPower(-1);
        robot.verticalElevator.setPower(0.2);
        sleep(1800);
        robot.backClaw.setPosition(0);
        robot.verticalElevator.setPower(0);
        robot.horizontalElevator.setPower(0);
        sleep(300);
        robot.horizontalElevator.setPower(1);
        sleep(700);
        robot.horizontalElevator.setPower(0);
        pidDrive(0.5,150,90,normal);

    }
}
