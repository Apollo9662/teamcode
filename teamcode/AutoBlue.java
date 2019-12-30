package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.GyroOperator.DriveMode.normal;
@Autonomous(name = "Autonomous blue", group = "apollo")

public class AutoBlue extends autonum {
    private vuforia vuforia = new vuforia();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, false);
        telemetry.addData("state ==> ", "Start");
        telemetry.update();
        waitForStart();
        robot.setCatchers(1);
        if (opModeIsActive()) {

            //          pidDrive(0.3,85,90,normal);
            pidDrive(0.2, 14, 0, normal);
            sleep(1000);

//            while (counterLeft < 150 && counterRight < 150 && counterCenter < 150) {
////                stonePosition = vuforia.process();
////                if (stonePosition > 0) {
////                    counterLeft++;
////                    counterRight = 0;
////                    counterCenter = 0;
////                } else if (stonePosition < 0) {
////                    counterLeft = 0;
////                    counterRight = 0;
////                    counterCenter++;
////                } else {
////                    counterLeft = 0;
////                    counterRight++;
////                    counterCenter = 0;
////                }
//                double y = vuforia.procces();
//                telemetry.addData("y = ",y);
//                sleep(50);
//                telemetry.update();
//            }
            double y = vuforia.procces(hardwareMap);
            if (y > 0 && y != 10000 ){
                //  pathRight(-1);
            } else if (y<0 ){
                //  pathCenter(-1);
            } else {
                // pathLeft(-1);
            }
            while (opModeIsActive()) {
                telemetry.addData("hosfd", y);

                telemetry.update();
//
//            }
                //  robot.frontClaw.setPosition(1);
                //  if (stonePosition == RIGHT){
                //      pathRight();
                //  }
                //  if (stonePosition == LEFT){
                //      pathLeft();
                //  }
                //  if (stonePosition == CENTER){
                //      pathCenter();
                //  }
                //
                //  //pathRight();
//        //    pathLeft();
                //  robot.setCatchers(1);
                //  pidDrive(0.4,50,90,normal);
                //  pidDrive(0.4,-30,90,normal);

            }


            //       while (counter < 5) {
            //       stonePosition = vuforia.process();
            //          if (vuforia.targetVisible) {
            //              telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
            //                      vuforia.translation.get(0) / vuforia.mmPerInch, vuforia.translation.get(1) / vuforia.mmPerInch,
            //                      vuforia.translation.get(2) / vuforia.mmPerInch);
            //          }
            //          if (stonePosition == lastStonePosition) {
            //              counter++;
            //          } else {
            //              counter = 1;
            //          }
            //          lastStonePosition = stonePosition;

            //          Log.d("counter", "value: " + counter);
            //          telemetry.update();
            //      }

            //   }

        }
    }
}
