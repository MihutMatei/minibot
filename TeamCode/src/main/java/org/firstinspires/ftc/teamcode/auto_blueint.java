package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.advanced.DetectionPipeline;
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
@Autonomous(name = "BLUEINT")
public class auto_blueint extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;
    DetectionPipeline detectionPipeline;
    private DcMotorEx slider;
    private DcMotorEx intake;
    private DcMotor carusel;
    private Servo cuva;
    private DcMotorEx rotire;
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
        rotire = hardwareMap.get(DcMotorEx.class,"rotire");
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
        int compensare=1;
        Trajectory allignWithHub = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-5,5,Math.toRadians(150)))

                .build();

        Trajectory forwardToHub = drive.trajectoryBuilder(allignWithHub.end())
                .forward(12+compensare)

                .build();

        TrajectorySequence goToWarehouse = drive.trajectorySequenceBuilder(forwardToHub.end())
                .lineToSplineHeading(new Pose2d(3.5,0,Math.toRadians(90)))
                .back(25)
                .addTemporalMarker(0.1, ()->
                { // intake
                    intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    intake.setPower(0.7);
                })
                .build();

        TrajectorySequence backward = drive.trajectorySequenceBuilder(forwardToHub.end())
                .lineToSplineHeading(new Pose2d(0,0,Math.toRadians(90)))
                .build();
        TrajectorySequence allignTofield = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(3.5,-30,Math.toRadians(90)))


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
        sleep(500);
        switch (zone)
        {
            case 1:
                slider.setTargetPosition(-200);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(0.6);
                rotire.setTargetPosition(-1800);
                rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotire.setPower(-0.8);
                compensare=10;
                break;
            case 2:
                slider.setTargetPosition(-1200);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slider.setPower(0.6);
                rotire.setTargetPosition(-1800);
                rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotire.setPower(-0.8);
                compensare=0;
                break;

            case 3:
                slider.setTargetPosition(-1500);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slider.setPower(0.6);
                rotire.setTargetPosition(-1550);
                rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotire.setPower(-0.8);
                compensare=0;
                break;
        }

        sleep(500);

        int counter = 0;
        ElapsedTime runtime = new ElapsedTime(0);;
        runtime.startTime();
        while(opModeIsActive()) {
            Pose2d curPos = drive.getPoseEstimate();

            allignWithHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(-5, 5, Math.toRadians(150)))

                    .build();

            if (counter > 0) {
                drive.followTrajectory(allignWithHub);

                slider.setTargetPosition(-1500);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slider.setPower(0.6);
                rotire.setTargetPosition(-1470);
                rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotire.setPower(-0.8);


                forwardToHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(10 - counter * 1.5)
                        .build();

            }
            if (zone == 1 && counter == 0) {
                slider.setTargetPosition(-100);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(-0.3);
                forwardToHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(7 - counter * 1.5 + compensare)
                        .build();
            }

            drive.followTrajectory(forwardToHub);

            cuva.setPosition(0.5); // drop cube

            intake.setPower(0);

            sleep(600);

            slider.setTargetPosition(-50);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            slider.setPower(-0.7);
            rotire.setTargetPosition(0);
            rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotire.setPower(0.6);

            sleep(600);

            goToWarehouse = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(3.5 + counter * 2 + compensare / 2, 0, Math.toRadians(90)))
                    .back(30.5)
                    .addTemporalMarker(0.1, () ->
                    { // intake
                        intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        intake.setPower(0.6);
                    })
                    .build();

            drive.followTrajectorySequence(goToWarehouse);

//            if(counter == 1 && runtime.time() > 23)
//                break;
            if(counter <2)
            {
            boolean breakfrom = false;

            while (opModeIsActive() && !breakfrom) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -0.1,
                                0,
                                0
                        )
                );

                drive.update();
                int c = 0;
                while (c < 30) {
                    if (color.red() > 60 && color.green() > 60) {
                        cuva.setPosition(0.09);
                        intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        intake.setPower(0.7);
                        breakfrom = true;
                        break;
                    }
                    sleep(5);
                    c++;
                }
            }
            intake.setPower(0);

            drive.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            0,
                            0
                    )
            );


            backward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(Math.abs(drive.getPoseEstimate().getY()))
                    .build();



                drive.followTrajectorySequence(backward);
        }

            if(counter==2)
            {    intake.setPower(0);
                //traiectorie

                TrajectorySequence endtraj=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .strafeLeft(28)
                        .turn(Math.toRadians(90))
                        .strafeLeft(25)
                        .build();
                drive.followTrajectorySequence(endtraj);
                break;
            }
            counter++;
        }

    }
}