package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class TeleOpCode extends LinearOpMode {
    //Defining the 4 drivetrain motors as null
    private DcMotorEx FrontLeftDT = null;
    private DcMotorEx FrontRightDT = null;
    private DcMotorEx BackLeftDT = null;
    private DcMotorEx BackRightDT = null;

    private DcMotorEx intakeMotor = null; //setting intake motor variable
    private Servo s1, s2;

    private DcMotorEx linearSlideLeft = null;
    private DcMotorEx linearSlideRight = null;




    @Override
    public void runOpMode() throws InterruptedException {
        //Calling the motor and defining them to the previous variables
        FrontLeftDT = hardwareMap.get(DcMotorEx.class, "leftFront");
        FrontRightDT = hardwareMap.get(DcMotorEx.class, "leftRear");
        BackLeftDT = hardwareMap.get(DcMotorEx.class, "rightFront");
        BackRightDT = hardwareMap.get(DcMotorEx.class, "rightRear");

        //setting left ones reverse since motors will be set up reverse
        FrontLeftDT.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftDT.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {
            /* ------- DRIVETRAIN ------- */
            //setting the directions onto the gamepad
            double axial = -gamepad1.left_stick_y;  // forward, back
            double lateral = gamepad1.left_stick_x; // side to side
            double yaw = gamepad1.right_stick_x; // turning

            //setting the directions all at 1 (100%)
            double axialCoefficient = 1;
            double yawCoefficient = 1;
            double lateralCoefficient = 1;

            //resetting the coefficient based on the coefficient that we can change
            yaw = yaw * yawCoefficient;
            axial = axial * axialCoefficient;
            lateral = lateral * lateralCoefficient;

            //slowmode?
            boolean slowModeOn = false;

            if (gamepad1.left_bumper) slowModeOn = true;

            double powerModifier = slowModeOn ? 0.3 : 0.8;

            //making the equations in order to have the directions synchronous
            double leftFrontPower = (axial + lateral + yaw) * powerModifier;
            double rightFrontPower = (axial - lateral - yaw) * powerModifier;
            double leftBackPower = (axial - lateral + yaw) * powerModifier;
            double rightBackPower = (axial + lateral - yaw) * powerModifier;

            //makes all our intial variables to the powers set before
            FrontLeftDT.setPower(leftFrontPower);
            FrontRightDT.setPower(rightFrontPower);
            BackLeftDT.setPower(leftBackPower);
            BackRightDT.setPower(rightBackPower);



            /* ------- INTAKE ------- */
            double power = gamepad2.right_stick_y;
            intakeMotor.setPower(power);

            /*--------OUTTAKE---------*/


             double SERVO_MAX = 1.0;
             double SERVO_MIN = 0.0;
             double servoPos1 = 1.0;
             double servoPos2 = 1.0;
            // if right bumper pressed first servo releases
            if(gamepad2.right_bumper){
                servoPos1 = SERVO_MIN;
            }
            // if left bumper pressed second servo releases
            else if(gamepad2.left_bumper){
                servoPos2 = SERVO_MIN;
            }
            s1.setPosition(servoPos1);
            s2.setPosition(servoPos2);

            // linear slide

            linearSlideLeft = hardwareMap.get(DcMotorEx.class, "LinearSlideLeft");

            linearSlideRight = hardwareMap.get(DcMotorEx.class, "LinearSlideRight");
            if(gamepad2.right_stick_y >= 0 ){
                linearSlideRight.setPower(0.6);
                linearSlideLeft.setPower(0.6);

            }

            else if(gamepad2.right_stick_y <= 0 ){
                linearSlideRight.setPower(-0.6);
                linearSlideLeft.setPower(-0.6);
            }


        }
    }
}