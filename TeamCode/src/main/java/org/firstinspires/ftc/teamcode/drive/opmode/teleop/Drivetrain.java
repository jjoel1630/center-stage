package org.firstinspires.ftc.teamcode.drive.opmode.teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp
public class Drivetrain extends LinearOpMode {
/* ________________________Variable Constants____________________*/

    //Defining the 4 drivetrain motors as null
    private DcMotorEx FrontLeftDT = null;
    private DcMotorEx FrontRightDT = null;
    private DcMotorEx BackLeftDT = null;
    private DcMotorEx BackRightDT = null;

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

        while (opModeIsActive()) {
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
        }
    }
}

