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

 import com.acmerobotics.roadrunner.geometry.Pose2d;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.ColorSensor;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorEx;
 import com.qualcomm.robotcore.hardware.DcMotorSimple;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.util.ElapsedTime;
 import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
 @TeleOp(name="MINIDRIVE", group="Linear Opmode")

 public class MiniDrive extends LinearOpMode {



     private DcMotorEx carusel;
     private DcMotorEx rotire;
     private Servo cleste;


     @Override
     public void runOpMode() throws InterruptedException {

         carusel =hardwareMap.get(DcMotorEx.class,"carusel");
         rotire =hardwareMap.get(DcMotorEx.class,"rotire");
         cleste =hardwareMap.get(Servo.class,"cleste");
         ElapsedTime runtime = new ElapsedTime(0);

         SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

         waitForStart();
         drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotire.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         rotire.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         while (opModeIsActive()) {
             rotire.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//
//             telemetry.addData("servo", cleste.getPosition());
//             telemetry.addData("rotire", rotire.getCurrentPosition());
//             telemetry.update();
             ElapsedTime runtime1 = new ElapsedTime(0);

             drive.setWeightedDrivePower(
                     new Pose2d(
                             gamepad1.left_stick_y*(0.5),
                             gamepad1.left_stick_x*(0.5) ,
                             -gamepad1.right_stick_x*(0.5)
                     )

             );
             drive.update();

             if(gamepad1.dpad_right)
             {
                 runtime.reset();
                 while(runtime.time() < 1.2 && !gamepad1.dpad_up) {
                     carusel.setPower(0.5);
                 }

                 carusel.setPower(0.75);
             }
             if(gamepad1.dpad_left)
             {
                 runtime.reset();
                 while(runtime.time() < 1.2 && !gamepad1.dpad_up) {
                     carusel.setPower(-0.5);
                 }

                 carusel.setPower(-0.75);
             }
            if(gamepad1.dpad_up)carusel.setPower(0);
             if(gamepad2.cross)
             {
                 rotire.setTargetPosition(-1300);
                 rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 rotire.setPower(-0.4);
             }
             if(gamepad2.square)
             {
                 rotire.setTargetPosition(-1600);
                 rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 rotire.setPower(-0.4);
             }
             if(gamepad2.circle)
             {
                 rotire.setTargetPosition(-1800);
                 rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 rotire.setPower(-0.4);
             }
             if(gamepad2.triangle)
             {
                 rotire.setTargetPosition(5);
                 rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 rotire.setPower(0.4);
             }
             if(gamepad2.dpad_down)
             {
                 rotire.setTargetPosition(-1770);
                 rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 rotire.setPower(-0.4);
             }
             if(gamepad2.dpad_up)
             {
                 rotire.setTargetPosition(-1150);
                 rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 rotire.setPower(-0.3);
             }
             if(gamepad2.right_bumper)
             {
                 rotire.setTargetPosition(-1250);
                 rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 rotire.setPower(-0.3);
             }

             //rotire.setPower(-gamepad2.left_stick_y/2);


             if(gamepad1.cross)
             {
                 cleste.setPosition(0.85);
             }
             if(gamepad1.square)
             {
                 cleste.setPosition(1);
             }
             telemetry.addData("Rotire",rotire.getCurrentPosition());
             telemetry.addData("cletste",cleste.getPosition());
             telemetry.update();

         }
     }
 }
