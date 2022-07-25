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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name = "Bluext")
public class auto_bluext extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;
    DetectionPipeline detectionPipeline;
    boolean bCameraOpened = false;
    private DcMotorEx carusel;
    private DcMotorEx rotire;
    private Servo cleste;

    //unfinished

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        carusel =hardwareMap.get(DcMotorEx.class,"carusel");
        rotire =hardwareMap.get(DcMotorEx.class,"rotire");
        cleste =hardwareMap.get(Servo.class,"cleste");



        carusel.setDirection(DcMotorSimple.Direction.REVERSE);




        pipeline = new SamplePipeline();
        detectionPipeline = new DetectionPipeline();
        webcam.setPipeline(detectionPipeline);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime runtime2 = new ElapsedTime(0);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);
        TrajectorySequence turnDuck = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-90))
                .back(25)
                .build();
        Trajectory alignwithhub = drive.trajectoryBuilder(turnDuck.end())
                .lineToSplineHeading( new Pose2d(-33,15,Math.toRadians(95)))
                .build();




        

        //x -9  y 7

        // x -1 y 20 pt duck

        // x -19 y 24 pt duck scan

        // x -37 y12 perpedincular cu shiphubu pt punerea duck

        // x -23 y 29 parking

        //----------------------------------------------------------------------------------------------


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

        detectionPipeline.setGridSize(2);

        double left_avg,right_avg;

        int zone = 0;
        int compensare =0;
        sleep(5000);
        while (!opModeIsActive() && !isStopRequested()) {
            //telemetry.addData("Zona", pipeline.getZone());


            left_avg = (detectionPipeline.getZoneLuminosity(1) + detectionPipeline.getZoneLuminosity(2)) / 2;
            right_avg = (detectionPipeline.getZoneLuminosity(3) + detectionPipeline.getZoneLuminosity(4)) / 2;
            //scimbat zona 1 cu zona 3 pt ca capera era poz gresit
            if (left_avg <= 126)
                zone = 3;
            else if (right_avg <= 122)
                zone = 2;
            else
                zone = 1;

            telemetry.addData("Zone", zone);
            telemetry.addData("Left", left_avg);
            telemetry.addData("Right", right_avg);

            telemetry.update();
        }

        rotire.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (!opModeIsActive()) return;

        cleste.setPosition(1);
        sleep(500);
        rotire.setTargetPosition(-50);
        rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotire.setPower(-0.5);
        sleep(500);
        drive.followTrajectorySequence(turnDuck);
        runtime2.reset();
        while(runtime2.time()<5)
        {
            carusel.setPower(0.45);
        }
        carusel.setPower(0);
        sleep(500);
        drive.followTrajectory(alignwithhub);
        switch (zone)
        {
            case 1:
            rotire.setTargetPosition(-1800);
            rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotire.setPower(-0.5);

                break;
            case 2:
            rotire.setTargetPosition(-1575);
            rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotire.setPower(-0.5);
            compensare=8;
                break;

            case 3:
            rotire.setTargetPosition(-1200);
            rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotire.setPower(-0.5);
            compensare=8;
                break;
        }
        sleep(500);
        Trajectory forwardtohub = drive. trajectoryBuilder(drive.getPoseEstimate())
                .back(34-compensare)
                .build();
        drive.followTrajectory(forwardtohub);
        sleep(500);
        cleste.setPosition(0.5);
        sleep(500);
        Trajectory realign = drive. trajectoryBuilder(drive.getPoseEstimate())
                .forward(20)
                .addTemporalMarker(0.5,()->{
                    rotire.setTargetPosition(0);
                    rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rotire.setPower(0.5);
                })
                .build();
        drive.followTrajectory(realign);
        sleep(500);
        Trajectory parkstorage = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-20,20))
                .build();
        drive.followTrajectory(parkstorage);








    }
}