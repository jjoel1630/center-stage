package org.firstinspires.ftc.teamcode.drive.opmode.tasks.TeamCode.src.main.java.org.firstinspires.ftc.teamcode.drive.opmode.tasks;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

    @Config
    @Autonomous(name="Sample Drive Train")

    public class DriveTrain extends LinearOpMode {
        DcMotorEx leftFront, leftRear;
        DcMotorEx rightFront, rightRear;

        public void runOpMode() throws InterruptedException {
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
            rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

            boolean axial = gamepad1.b;
            boolean lateral = gamepad1.y;
            boolean yaw = gamepad1.x;

            waitForStart();

            while (opModeIsActive() && !isStopRequested()) {
                goForward(0.25, 5000); //x distance
                stopMotors();
                turnR(0.20, 3000); //180 degrees
                stopMotors();
                goForward(-0.25, 5000); //x distance back
                stopMotors();
                turnR(0.20, 2000);//90 degrees
                stopMotors();
                strafe(0.25, 5000); //strafe
                stopMotors();
                stop();

            }
        }

        public void goForward(double Power, long time){
            leftFront.setPower(Power);
            leftRear.setPower(Power);
            rightFront.setPower(Power);
            rightRear.setPower(Power);

            sleep(time);
        }
        public void turnR (double Power, long time){
            leftFront.setPower(Power);
            leftRear.setPower(Power);
            rightFront.setPower(-Power);
            rightRear.setPower(-Power);

            sleep(time);
        }

        public void stopMotors(){
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }

        public void strafe (double Power, long time){
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(Power)
                    .build();
            sleep(time);
        }
    }
