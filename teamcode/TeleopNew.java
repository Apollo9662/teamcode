/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

        package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.functions;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.MathFunctions.map;


/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleopNe", group="APOLLO")
public class TeleopNew extends functions {
    Hardware robot = new Hardware();
    private Thread slidesOperation = new slidesOperation();
    private Thread clawOperation = new clawOperation();
    private Thread catchers = new catchers();
    private Thread stoneLevel = new stoneLevel();
    private InputOperation inputOperation = new InputOperation();
    private double collectionSpeed = 1;

    private boolean isOpen = false;
    private boolean isOpen2 = false;


    double diameterPulleySlideSide = 1;
    double diameterPulleySlideUp = 1;

    private static final double countsPer20gearmotor = 537.6;    // eg: TETRIX Motor Encoder
    private static final double pulleyDiameter = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH_FOR_PULLEY = (countsPer20gearmotor) / (pulleyDiameter * Math.PI);

    private static final long threadSleepTimeOut = 50; // 50 msec

    private static final double ticksPerLevel = 30.0;
    private static final int maxLevel = 7;
    private static final double maxElevatorValue = countsPer20gearmotor * (ticksPerLevel * maxLevel);

    private int currentLevel = 0;

    public void runOpMode() {


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap,true);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        waitForStart();

        clawOperation.start();
        slidesOperation.start();
        inputOperation.start();
        catchers.start();
        stoneLevel.start();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("left front", robot.driveLeftFront.getCurrentPosition() );
            telemetry.addData("left back", robot.driveLeftBack.getCurrentPosition() );
            telemetry.addData("right front", robot.driveRightFront.getCurrentPosition() );
            telemetry.addData("right back", robot.driveRightBack.getCurrentPosition() );
            telemetry.addData("vertical", robot.verticalElevator.getCurrentPosition() );
            telemetry.update();
            driveOperation();

        }
    }


    private class clawOperation extends Thread{
        clawOperation(){ this.setName("clawOperation");}
        @Override
        public void run(){
            try {
                while (!isInterrupted() && opModeIsActive()){
                    while (!isInterrupted() && opModeIsActive()){
                        double frontMaxPosition = 0;
                        double frontMinPosition = 1;

                        double backMaxPosition = 1;
                        double backMinPosition = 0;

                        double backPosition;
                        double frontPosition;
                        if(gamepad2.left_trigger > 0.1){
                            frontPosition = frontMinPosition;
                        }else{
                            frontPosition = frontMaxPosition;
                        }
                        if(gamepad2.right_trigger > 0.1){
                            telemetry.addData("ga",gamepad2.right_trigger);
                            backPosition = backMinPosition;
                        }else{
                            backPosition = backMaxPosition;
                        }
                        if(gamepad1.left_trigger > 0.1){
                            frontPosition = frontMinPosition;
                        }
                        robot.frontClaw.setPosition(frontPosition);
                        robot.backClaw.setPosition(backPosition);
                    }
                }
                Thread.sleep(threadSleepTimeOut);
            }
            catch (InterruptedException ignored){}
        }
    }



    private class slidesOperation extends Thread {
        slidesOperation() { this.setName("slidesOperation");}

        @Override
        public void run() {
            try {
                while (!isInterrupted() && opModeIsActive()) {
                    double topLimitSlideSide = 916.75950;
//                       if (robot.horizontalElevator.getCurrentPosition() > topLimitSlideSide || robot.horizontalElevator.getCurrentPosition() < 0) {
//                           robot.horizontalElevator.setPower(0);
//                       } else {
//                           robot.horizontalElevator.setPower(gamepad2.right_stick_y);
//                       }
                    if(gamepad2.right_stick_x > 0.1){
                        robot.horizontalElevator.setPosition(0.8);
                    }else if(gamepad2.right_stick_x < -0.1){
                        robot.horizontalElevator.setPosition(0.2);
                    }else{
                        robot.horizontalElevator.setPosition(0);
                    }
                    robot.horizontalElevator.setPosition(gamepad2.right_stick_x);
                    robot.verticalElevator.setPower(gamepad2.left_stick_y);
                    telemetry.addData("dakhghjdf",gamepad2.left_stick_y);
                }
                Thread.sleep(threadSleepTimeOut);
            } catch (InterruptedException ignored) {}
        }
    }


    public void driveOperation()  {
        double turn = -gamepad1.right_stick_x * 0.35;
        double right_x = gamepad1.left_stick_x;
        double right_y = gamepad1.left_stick_y;

        double vector = Math.hypot(right_x,right_y);
        double angle = Math.toDegrees(Math.atan2(right_y,right_x));
        angle += 135;

        right_x = vector * Math.cos(Math.toRadians(angle));
        right_y = vector * Math.sin(Math.toRadians(angle));



        robot.driveLeftFront.setPower(right_y - turn);
        robot.driveRightBack.setPower(right_y + turn);
        robot.driveLeftBack.setPower(right_x - turn);
        robot.driveRightFront.setPower(right_x + turn);
    }

    private class reverseDrive extends Thread {
        reverseDrive() {
            this.setName("reverseDrive");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {
            try {
                while (!isInterrupted() && opModeIsActive()) {
                    while (!isInterrupted() && opModeIsActive()) {
                        if (gamepad1.left_bumper) {
                            robot.reverse();
                            while (gamepad1.left_bumper && opModeIsActive());
                        }
                    }
                    Thread.sleep(threadSleepTimeOut);
                }
            } catch (InterruptedException ignored) {
            }
        }
    }


    private class InputOperation extends Thread {
        InputOperation() {
            this.setName("InputOperation");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {
            try {
                while (!isInterrupted() && opModeIsActive()) {

                    if(gamepad1.left_trigger > 0.2) {
                        robot.setCollectMotorsPower(collectionSpeed * gamepad1.left_trigger);
                    }
                    if(gamepad1.right_trigger > 0.2) {
                        robot.setCollectMotorsPower(-collectionSpeed * gamepad1.right_trigger);
                    }else if(gamepad1.left_trigger < 0.2){
                        robot.setCollectMotorsPower(0);
                    }


                    if (gamepad1.right_bumper) {
                        collectionSpeed = 0.3;
                    } else {
                        collectionSpeed = 1;
                    }
                    Thread.sleep(threadSleepTimeOut);
                }
            } catch (InterruptedException ignored) {
            }
        }
    }


    private class stoneLevel extends Thread{
        stoneLevel(){this.setName("stoneLevel");}

        @Override
        public void run(){
            try {
                while (!isInterrupted() && opModeIsActive()) {
                    if (gamepad2.left_bumper) {
                        if (currentLevel < 7) {
                            currentLevel = currentLevel + 1;
                            telemetry.addData("", "current level +");
                        }
                        while (gamepad2.left_bumper && opModeIsActive()) ;
                    }
                    if (gamepad2.right_bumper) {
                        if (currentLevel > 0) {
                            currentLevel = currentLevel - 1;
                            telemetry.addData("", "current level -");
                        }
                        while (gamepad2.left_bumper && opModeIsActive()) ;
                    }
                    if (gamepad2.left_trigger > 0.6) {
                        robot.verticalElevator.setTargetPosition((int) countsPer20gearmotor * 5 * currentLevel);
                    }
                    Thread.sleep(threadSleepTimeOut);
                }
            }catch (InterruptedException ignored){

            }
        }
    }
    private class elevatorControl extends Thread{
        elevatorControl(){this.setName("elevatorControl");}

        @Override
        public void run(){
            try {
                while(!isInterrupted() && opModeIsActive()) {
                    double verticalRatio = gamepad2.left_stick_y;
                    double verticalPosition = abs(verticalRatio) * maxElevatorValue;

                    robot.verticalElevator.setTargetPosition((int) verticalPosition);
//                robot.verticalElevator.gotoPosition(0.6);

                    double horizontalPower = gamepad2.right_stick_x;
                    //robot.horizontalElevator.setPower(horizontalPower);

                    Thread.sleep(threadSleepTimeOut);
                }
            }catch (InterruptedException ignored){

            }
        }
    }
    private class catchers extends Thread{
        int position = 0;
        catchers(){setName("catchers");}
        @Override
        public void run(){
            while(opModeIsActive() && !isInterrupted()){
                if(gamepad1.y){
                    position = position == 1? 0 : 1;
                    while (gamepad1.y && opModeIsActive());
                }
                robot.setCatchers(position);
            }
        }
    }
}





