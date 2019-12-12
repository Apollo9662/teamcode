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

import static java.lang.Math.abs;

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

@TeleOp(name="Teleop", group="APOLLO")
public class Teleop extends functions {

    double speedFactor = 1;
    Hardware robot = new Hardware();
    Thread reverseDrive = new reverseDrive();
    Thread driveOperation = new driveOperation();
    Thread slidesOperation = new slidesOperation();
    Thread clawOperation = new clawOperation();
    InputOperation inputOperation = new InputOperation();
    double collectionSpeed = 1;

    private boolean isOpen = false;
    private boolean isOpen2 = false;


    double diameterPulleySlideSide = 1;
    double diameterPulleySlideUp = 1;

    double topLimitSlideUp = 2750.278529;
    double topLimitSlideSide = 916.75950;

    static final double countsPerUpSlide = 537.6;    // eg: TETRIX Motor Encoder
    static final double  pullyDiameter = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (countsPerUpSlide) / (pullyDiameter * Math.PI);

    static final long threadSleepTimeOut = 50; // 50 msec

    int correnctLevel = 0;
    public void runOpMode() {


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        waitForStart();


        driveOperation.start();
        clawOperation.start();
        slidesOperation.start();
        reverseDrive.start();
        inputOperation.start();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Robot  X", robot.getX());
            telemetry.addData("Robot  Y", robot.getY());

            telemetry.update();
        }
    }


    private class clawOperation extends Thread{
        public clawOperation(){ this.setName("clawOperation");}
        @Override
        public void run(){
            try {
                while (opModeIsActive()){
                    while (!isInterrupted() && opModeIsActive()){
                        if (gamepad2.b) {
                            if (isOpen) {
                                robot.outPutCloseServo.setPosition(0.05);
                                isOpen = false;
                                while (gamepad2.b == false) {
                                    Thread.sleep(threadSleepTimeOut);
                                }
                            } else {
                                robot.outPutCloseServo.setPosition(0.75);
                                isOpen = true;
                                while (gamepad2.b == false) {
                                    Thread.sleep(threadSleepTimeOut);
                                }
                            }
                        }

                        if (gamepad2.a) {
                            if (isOpen2) {
                                robot.outPutCloseServo.setPosition(0.05);
                                isOpen2 = false;
                                while (gamepad2.a == false) {
                                    Thread.sleep(threadSleepTimeOut);
                                }
                            } else {
                                robot.outPutCloseServo.setPosition(0.75);
                                isOpen2 = true;
                                while (gamepad2.a == false) {
                                    Thread.sleep(threadSleepTimeOut);
                                }
                            }
                        }

                        if (gamepad2.dpad_left || gamepad2.dpad_right) {
                            robot.turret.setPosition(0.05);
                        }
                        if (gamepad2.dpad_down || gamepad2.dpad_up) {
                            robot.turret.setPosition(0.3);
                        }
                    }
                }
                Thread.sleep(5);
            }
            catch (InterruptedException e){}
        }
    }



    private class slidesOperation extends Thread {
        public slidesOperation() { this.setName("slidesOperation");}

        @Override
        public void run() {
            try {
                while (opModeIsActive()) {
                    while (!isInterrupted() && opModeIsActive()) {
                        if (robot.elevatorSide.getCurrentPosition() > topLimitSlideSide || robot.elevatorSide.getCurrentPosition() < 0) {
                            robot.elevatorSide.setPower(0);
                        } else {
                            robot.elevatorSide.setPower(gamepad2.right_stick_y);
                        }

                        if (robot.elevatorUp.getCurrentPosition() > topLimitSlideUp || robot.elevatorUp.getCurrentPosition() < 0) {
                            robot.elevatorUp.setPower(0);

                        } else {
                            robot.elevatorUp.setPower(gamepad2.left_stick_y);
                        }
                    }
                }
                Thread.sleep(threadSleepTimeOut);
            } catch (InterruptedException e) {}
        }
    }


        private class driveOperation extends Thread {
            public driveOperation() {
                this.setName("driveOperation");
            }

            @Override
            public void run() {
                try {
                    while (opModeIsActive()) {
                        while (!isInterrupted() && opModeIsActive()) {
                            double left_x = gamepad1.left_stick_x;
                            double left_y = -gamepad1.left_stick_y;
                            double right_x = gamepad1.right_stick_x;
                            double right_y = -gamepad1.right_stick_y;

                            if (left_x < -0.05 && abs(left_y) > 0.05 && right_x < -0.05 && abs(right_y) > 0.05) {
                                robot.setDriveMotorsPower(left_y * speedFactor, Hardware.DRIVE_MOTOR_TYPES.DIAGONAL_LEFT);
                                telemetry.addData("", "DIAGONAL_LEFT");

                            } else if (left_x > 0.05 && abs(left_y) > 0.05 && right_x > 0.05 && abs(right_y) > 0.05) {
                                robot.setDriveMotorsPower(left_y * speedFactor, Hardware.DRIVE_MOTOR_TYPES.DIAGONAL_RIGHT);
                                telemetry.addData("", "DIAGONAL_RIGHT");

                            } else if (left_x < -0.05 && right_x < -0.05) {
                                robot.setDriveMotorsPower(left_x, Hardware.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                                telemetry.addData("", "LEFT");

                            } else if (left_x > 0.05 && right_x > 0.05) {
                                robot.setDriveMotorsPower(left_x, Hardware.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                                telemetry.addData("", "RIGHT");

                            } else {
                                robot.setDriveMotorsPower(left_y, Hardware.DRIVE_MOTOR_TYPES.LEFT);
                                robot.setDriveMotorsPower(right_y, Hardware.DRIVE_MOTOR_TYPES.RIGHT);
                                telemetry.addData("", "TANK");


                            }
                            telemetry.addData("X position", robot.getX());
                            telemetry.addData("Y position", robot.getY());

                        }
                    }
                    Thread.sleep(threadSleepTimeOut);
                } catch (InterruptedException e) {
                }

            }
        }

        private class reverseDrive extends Thread {
            public reverseDrive() {
                this.setName("reverseDrive");
            }

            // called when tread.start is called. thread stays in loop to do what it does until exit is
            // signaled by main code calling thread.interrupt.
            @Override
            public void run() {
                try {
                    while (opModeIsActive()) {
                        while (!isInterrupted() && opModeIsActive()) {
                            if (gamepad1.left_bumper) {
                                robot.reverse();
                                while (gamepad1.left_bumper && opModeIsActive()) {
                                    continue;
                                }
                            }
                        }
                        Thread.sleep(threadSleepTimeOut);
                    }
                } catch (InterruptedException e) {
                }
            }
        }


    private class InputOperation extends Thread {
        public InputOperation() {
            this.setName("InputOperation");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {
            try {
                while (opModeIsActive()) {

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
                    Thread.sleep(10);
                }
            } catch (InterruptedException e) {
            }
        }
    }


        private class stoneLevel extends Thread{
        public stoneLevel(){this.setName("stoneLevel");}

        @Override
            public void run(){
            try {
                while (opModeIsActive()){
                    while (!isInterrupted() && opModeIsActive()){
                        if (gamepad2.left_bumper){
                           if (correnctLevel<7) {
                               correnctLevel = correnctLevel + 1;
                           }
                        } else {
                            if (gamepad2.right_bumper){
                                if (correnctLevel>0) {
                                    correnctLevel = correnctLevel - 1;
                                }
                            }
                        }
                        if (gamepad2.left_trigger>0.6){
                            robot.elevatorUp.setTargetPosition((int)COUNTS_PER_INCH*5*correnctLevel);

                        }

                    }
                    Thread.sleep(threadSleepTimeOut);
                }

            }catch (InterruptedException e){

            }
        }

        }

}





