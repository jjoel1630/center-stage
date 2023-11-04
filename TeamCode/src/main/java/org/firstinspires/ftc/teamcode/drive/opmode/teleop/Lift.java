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
    public void runOpMode() throws InterruptedException {
        //declaring motors for linear slides
        liftMotorOne = hardwareMap.get(DcMotorEx.class, "LiftOne");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, "LiftTwo");
        liftMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        int encoderValueOne = liftMotorOne.getCurrentPosition(); //creating variables for encoder positioning
        int encoderValueTwo = liftMotorTwo.getCurrentPosition();

        int stageZero = 0; //to go all the way down
        int stageOne = 10; //encoder ticks needed to reach each level
        int stageTwo = 20;
        int stageThree = 30;


        waitForStart();

        liftMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //to count encoder ticks
        liftMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION); //able to go to certain position
        liftMotorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        encoderValueOne = 0; //setting encoder values to 0
        encoderValueTwo = 0;

        double motor_tick_count = 1; //find ticks counts


        while (opModeIsActive()) {

            double power = gamepad2.left_stick_y; //to control linear slides manually
            liftMotorOne.setPower(power);
            liftMotorTwo.setPower(power);

            encoderValueOne = liftMotorOne.getCurrentPosition(); //get current encoder position
            encoderValueTwo = liftMotorTwo.getCurrentPosition();

            //buffers
            if (encoderValueOne <= 0) { //setting power to 0 if encoder values reach 0
                power = 0;
            }

            if (encoderValueTwo <= 0) {
                power = 0;
            }

            if (encoderValueOne == stageThree) { //setting power to 0 if encoder values reach highest stage
                power = 0;
            }
            if (encoderValueTwo == stageThree) {
                power = 0;



                if (gamepad2.dpad_left) {
                    liftMotorOne.setPositionPIDFCoefficients(stageOne);
                    liftMotorOne.setPower(1);
                    liftMotorTwo.setPositionPIDFCoefficients(stageOne);
                    liftMotorTwo.setPower(1);
                }

                if (gamepad2.dpad_up) {
                    liftMotorOne.setPositionPIDFCoefficients(stageTwo);
                    liftMotorOne.setPower(1);
                    liftMotorTwo.setPositionPIDFCoefficients(stageTwo);
                    liftMotorTwo.setPower(1);
                }

                if (gamepad2.dpad_right) {
                    liftMotorOne.setPositionPIDFCoefficients(stageThree);
                    liftMotorOne.setPower(1);
                    liftMotorTwo.setPositionPIDFCoefficients(stageThree);
                    liftMotorTwo.setPower(1);
                }

                if (gamepad2.dpad_down) {
                    liftMotorOne.setPositionPIDFCoefficients(stageZero);
                    liftMotorOne.setPower(1);
                    liftMotorTwo.setPositionPIDFCoefficients(stageZero);
                    liftMotorTwo.setPower(1);
                }




                }
            }

        }

    }

}