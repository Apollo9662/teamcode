package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous(name = "ToPointTest",group = "Apollo")
public class autonomous extends functions {

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("state", "waiting");
        telemetry.addData("version", "1.0");
        telemetry.update();
        waitForStart();
        //turn(0.5,180);
        //driveToPosition();
        while (opModeIsActive()) {
            telemetry.addData("", robot.driveRightBack.getCurrentPosition());
            telemetry.update();
        }


    }
}