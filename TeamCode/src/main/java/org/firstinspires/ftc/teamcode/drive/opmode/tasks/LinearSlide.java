package org.firstinspires.ftc.teamcode.drive.opmode.tasks;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.opmode.bot.PowerPlayBot;
//import org.firstinspires.ftc.teamcode.drive.opmode.imagedetect.SleeveColorDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="Linear Slide Task")
public class LinearSlide extends LinearOpMode {
    private ElapsedTime timer;

    public int POSITION = 1;

    public int strafeInches = 40;
    public int forwardInches = 27;

    public static int ticks = 50;
    public static int initticks = 50;
    public static double time = 5;
    public static int numTimes = 10;
    public static double pwr = 0.8;

    Vector2d storage = new Vector2d(63.82, -12.2);
    Pose2d storagep = new Pose2d(63.52, -11.90, 0);

    DcMotorEx linearSlide;

    public double x = 21.0; // 19.00
    public double y = -6.0; // 4.01   //what is this for??

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive.setPoseEstimate(new Pose2d(32.81, -64.87, Math.toRadians(90.00)));

        TrajectorySequence scorePreloaded = drive.trajectorySequenceBuilder(new Pose2d(32.81, -64.87, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(12.62, -53.59), Math.toRadians(90.00))
                .splineTo(new Vector2d(12.62, -17.52), Math.toRadians(90.00))
                .splineTo(new Vector2d(x, y), Math.toRadians(45.00))
                .build();

        TrajectorySequence positionPark1 = drive.trajectorySequenceBuilder(new Pose2d(x, y, Math.toRadians(45.00)))
                .lineToConstantHeading(new Vector2d(11.88, -11.28))
                .build();

        TrajectorySequence positionPark2 = drive.trajectorySequenceBuilder(new Pose2d(11.88, -11.28, Math.toRadians(45.00)))
                .lineToLinearHeading(new Pose2d(12.92, -36.15, Math.toRadians(90.00)))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence park0 = drive.trajectorySequenceBuilder(new Pose2d(12.92, -36.15, Math.toRadians(180)))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(new Pose2d(12.92, -36.15, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(36.22, -36.15))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(new Pose2d(12.92, -36.15, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(59.68, -36.15))
                .build();
        };


        }

