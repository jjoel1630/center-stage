package org.firstinspires.ftc.teamcode.drive.opmode.teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp
public class Lift extends LinearOpMode {
    private DcMotorEx liftMotorOne = null; //ask joel why this is done
    private DcMotorEx liftMotorTwo = null;

    @Override
    public void runOpMode() throws InterruptedException{
        liftMotorOne = hardwareMap.get(DcMotorEx.class,"LiftOne");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class,"LiftTwo");
        liftMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        int encoderValueOne = liftMotorOne.getCurrentPosition();
        int encoderValueTwo = liftMotorTwo.getCurrentPosition();

        int stageOne = 10; //encoder ticks needed to reach each level
        int stageTwo = 20;
        int StageThree = 30;


        waitForStart();

        while(opModeIsActive()){
            double power = gamepad2.left_stick_y;

            liftMotorOne.setPower(power);
            liftMotorTwo.setPower(power);

            encoderValueOne = 0;
            encoderValueTwo = 0;

            //encoder code to make buffer
            //don't let linear slides go below/above certain point

        }
    }

}
