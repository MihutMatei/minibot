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

@Autonomous(name = "redint")
public class auto_redint extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(11, -59, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence aliniere1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-11.5,-43,Math.toRadians(-90)))
                .build();


        TrajectorySequence iacub = drive.trajectorySequenceBuilder(aliniere1.end())
                .lineToLinearHeading(new Pose2d(11,-59,Math.toRadians(0)))
                .lineTo(
                        new Vector2d(50, -59),
                        SampleMecanumDrive.getVelocityConstraint(10,90, 90),
                        SampleMecanumDrive.getAccelerationConstraint(10)
                )
                .addTemporalMarker(0.3,()->
                {
                    cleste.setPosition(0.85);
                    rotire.setTargetPosition(5);
                    rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rotire.setPower(-0.5);
                })
                .build();
        TrajectorySequence aliniere2 =drive.trajectorySequenceBuilder(iacub.end())
                .lineTo(new Vector2d(11,-63))
                .lineToLinearHeading(new Pose2d(-11.5,-43,Math.toRadians(-90)))
                .addTemporalMarker(0.3,()->
                {
                    rotire.setTargetPosition(-1200);
                    rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rotire.setPower(-0.5);
                })
                .build();
        TrajectorySequence parcare = drive.trajectorySequenceBuilder(aliniere2.end())
                .lineToLinearHeading(new Pose2d(11,-63,Math.toRadians(0)))
                .lineTo(new Vector2d(50, -59))
                .addTemporalMarker(0.1,()->
                {
                    rotire.setTargetPosition(5);
                    rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rotire.setPower(-0.5);
                })
                .build();

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
            else if (right_avg <= 126)
                zone = 2;
            else
                zone = 1;

            telemetry.addData("Zone", zone);
            telemetry.addData("Left", left_avg);
            telemetry.addData("Right", right_avg);

            telemetry.update();
        }
        cleste.setPosition(1);
        sleep(1000);
        drive.followTrajectorySequence(aliniere1);
        sleep(1000);
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

                break;

            case 3:
                rotire.setTargetPosition(-1200);
                rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotire.setPower(-0.5);

                break;
        }
        sleep(1000);
        cleste.setPosition(0.85);
        sleep(1000);
//        cleste.setPosition(1);
//        sleep(500);
//        drive.followTrajectorySequence(iacub);
//        cleste.setPosition(1);
//        drive.followTrajectorySequence(aliniere2);
//        cleste.setPosition(0.85);
//        sleep(500);
//        cleste.setPosition(1);
//        sleep(500);
        drive.followTrajectorySequence(parcare);








    }
}