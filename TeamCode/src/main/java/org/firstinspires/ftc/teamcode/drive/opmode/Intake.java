package org.firstinspires.ftc.teamcode.drive.opmode;
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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp

public class Intake extends LinearOpMode {
    private DcMotorEx IntakeMotor = null; //setting intake motor variable


@Override
    public void runOpMode() throws InterruptedException{
    IntakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
    //IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    IntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

}

    public void waitForStart() {

    while (opModeIsActive()) {
        IntakeMotor.setPower(1); //max = 1?

        double power = gamepad1.right_stick_y;
        double neg_power = -gamepad1.right_stick_y;

        if(power == 1) {
            IntakeMotor.setPower(1);
        }
        if (neg_power == 1) {
            IntakeMotor.setPower(-1);
        }
        if (gamepad1.a) {
            IntakeMotor.setPower(1);
        }


    }

        }
}
