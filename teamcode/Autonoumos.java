package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.GyroOperator.DriveMode.left;
import static org.firstinspires.ftc.teamcode.GyroOperator.DriveMode.normal;


@Autonomous(name = "auto")
public class Autonoumos extends GyroOperator{
    @Override
    public void runOpMode()  {

        robot.init(hardwareMap);
        telemetry.addData("state ==> ","Start");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
//            pathNormal();
            pathSide();
            telemetry.addData("state ==> ","Finish");
            telemetry.update();
        }
    }
    public void pathNormal(){
        drive(0.7,23,0, normal);

        sleep(300);

        drive(-0.7,8,0, normal);

        sleep(300);

        gyroTurn(0.3,-90);
        drive(0.7,45,-90, normal);
        robot.setCollectMotorsPower(1);

        sleep(300);

        drive(-0.7,10,-90, normal);
    }
    public void pathSide(){
        drive(0.7,32,0,left);
        for(int i = 0; i < 5 && opModeIsActive(); i++){
            sleep(2000);//closse servo

            drive(-0.7,50,0, left);
            drive(0.7,45,0, normal);

            sleep(2000);//open servo

            drive(-0.7,45,0, normal);
            drive(0.7,5,0,left);

        }
        drive(0.7,10,0, normal);

    }
}
