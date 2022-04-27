package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.advanced.DetectionPipeline;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.drive.advanced.SamplePipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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
@Autonomous(name = "bluint_test")
@Disabled
public class auto_bluet2 extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;
    DetectionPipeline detectionPipeline;
    private DcMotorEx slider;
    private DcMotorEx intake;
    private DcMotor carusel;
    private Servo cuva;
    private Servo rotire;
    boolean bCameraOpened = false;
    private ColorSensor color;

    //unfinished

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        slider = hardwareMap.get(DcMotorEx.class, "slider");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        carusel = hardwareMap.get(DcMotor.class, "carusel");
        cuva = hardwareMap.get(Servo.class,"cuva");
        rotire = hardwareMap.get(Servo.class,"rotire");
        color = hardwareMap.get(ColorSensor.class, "color");

        carusel.setDirection(DcMotorSimple.Direction.REVERSE);

        pipeline = new SamplePipeline();
        detectionPipeline = new DetectionPipeline();
        webcam.setPipeline(detectionPipeline);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                bCameraOpened = true;
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        Trajectory allignWithHub = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-5,5,Math.toRadians(150)))

                .build();

        Trajectory forwardToHub = drive.trajectoryBuilder(allignWithHub.end())
                .forward(16)

                .build();
        Trajectory forwardToHub2 = drive.trajectoryBuilder(allignWithHub.end())
                .forward(13)

                .build();
        Trajectory forwardToHub3 = drive.trajectoryBuilder(allignWithHub.end())
                .forward(15)

                .build();



        detectionPipeline.setGridSize(2);

        double left_avg,right_avg;

        int zone = 0;
        sleep(5000);
        while (!opModeIsActive() && !isStopRequested()) {
            //telemetry.addData("Zona", pipeline.getZone());
            cuva.setPosition(0.09);

            left_avg = (detectionPipeline.getZoneLuminosity(1) + detectionPipeline.getZoneLuminosity(2)) / 2;
            right_avg = (detectionPipeline.getZoneLuminosity(3) + detectionPipeline.getZoneLuminosity(4)) / 2;

            if (left_avg <= 125)
                zone = 1;
            else if (right_avg <= 125)
                zone = 2;
            else
                zone = 3;

            telemetry.addData("Zone", zone);
            telemetry.addData("Left", left_avg);
            telemetry.addData("Right", right_avg);

            telemetry.update();
        }
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (!opModeIsActive()) return;

        drive.followTrajectory(allignWithHub);

        switch (zone)
        {
            case 1:
                slider.setTargetPosition(-200);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(0.6);
                rotire.setPosition(0.95);
                sleep(500);
                drive.followTrajectory(forwardToHub);
                break;
            case 2:
                slider.setTargetPosition(-1200);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slider.setPower(0.6);
                rotire.setPosition(0.95);
                sleep(500);
                drive.followTrajectory(forwardToHub2);
                break;

            case 3:
                slider.setTargetPosition(-1500);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slider.setPower(0.6);
                rotire.setPosition(0.75);
                sleep(500);
                drive.followTrajectory(forwardToHub3);
                break;
        }

        sleep(200);
        cuva.setPosition(0.5); // drop cube
        sleep(800);

        intake.setPower(0);
        cuva.setPosition(0.09);
        sleep(500);

        slider.setTargetPosition(-50);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(-0.7);
        rotire.setPosition(0.03);
        sleep(400);


        cuva.setPosition(0.5); // drop cube
        intake.setPower(0);

        sleep(500);

        slider.setTargetPosition(-50);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        slider.setPower(-0.7);
        rotire.setPosition(0.03);

        sleep(100);

        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(-0.2);


        TrajectorySequence goToWarehouse = drive.trajectorySequenceBuilder(forwardToHub.end())
                .lineToSplineHeading(new Pose2d(4.5,0,Math.toRadians(90)))
                .back(35)
                .addTemporalMarker(0.1, ()->
                { // intake
                    intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    intake.setPower(0.5);
                })
                .build();


        while(opModeIsActive()) {
            drive.followTrajectorySequence(goToWarehouse);

            while (opModeIsActive()) {
                if (color.red() > 30 && color.green() > 30) {
                    cuva.setPosition(0.09);
                    intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    intake.setPower(0.4);
                    break;
                }

                drive.setWeightedDrivePower(
                        new Pose2d(
                                -0.05,
                                0,
                                0
                        )
                );
                drive.update();
                sleep(100);
            }

            TrajectorySequence backward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(4.5, 0, Math.toRadians(90)))
                    .build();

            sleep(1000);

            intake.setPower(0);

            drive.followTrajectorySequence(backward);

            sleep(500);

            allignWithHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(-5, 5, Math.toRadians(150)))

                    .build();

            drive.followTrajectory(allignWithHub);

            slider.setTargetPosition(-1500);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slider.setPower(0.6);
            rotire.setPosition(0.75);


            drive.followTrajectory(forwardToHub3);

            cuva.setPosition(0.5); // drop cube
            sleep(800);

            intake.setPower(0);
            cuva.setPosition(0.09);

            slider.setTargetPosition(-50);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(-0.7);
            rotire.setPosition(0.03);

            sleep(300);

            cuva.setPosition(0.5); // drop cube

            telemetry.speak("sveps cu mata");
        }


    }
}
