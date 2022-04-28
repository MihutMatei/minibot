package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.DetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.drive.advanced.SamplePipeline;
import org.firstinspires.ftc.teamcode.drive.advanced.DetectionPipeline;

@Autonomous(name = "BLUEXT")
public class auto_bluext extends LinearOpMode {



    private DcMotorEx carusel;
    private DcMotorEx rotire;
    private Servo cleste;


    //unfinished

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        carusel =hardwareMap.get(DcMotorEx.class,"carusel");
        rotire =hardwareMap.get(DcMotorEx.class,"rotire");
        cleste =hardwareMap.get(Servo.class,"cleste");

        carusel.setDirection(DcMotorSimple.Direction.REVERSE);
        rotire.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime runtime2 = new ElapsedTime(0);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        Trajectory punecub = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(40,-30,Math.toRadians(-135)))
                .build();




        //x -9  y 7

        // x -1 y 20 pt duck

        // x -19 y 24 pt duck scan

        // x -37 y12 perpedincular cu shiphubu pt punerea duck

        // x -23 y 29 parking

        //----------------------------------------------------------------------------------------------
        while (!opModeIsActive() && !isStopRequested()) {
        }

        if (!opModeIsActive()) return;
        drive.followTrajectory(punecub);
        //drive.followTrajectory(turnDuck);
//        runtime2.reset();
//        while(runtime2.time()<3)
//        {
//            carusel.setPower(0.4);
//        }



        sleep(2000);

    }
}