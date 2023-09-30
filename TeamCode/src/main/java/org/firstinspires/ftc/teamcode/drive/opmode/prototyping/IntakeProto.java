package org.firstinspires.ftc.teamcode.drive.opmode.prototyping;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="IntakePrototype")
public class IntakeProto extends LinearOpMode {
    public static int reverseRC = 1;
    public static int reverseRR = 1;
    public static int reverseLC = -1;
    public static int reverseLR = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx rightMotorFar = hardwareMap.get(DcMotorEx.class, "rightMotorFar");
        DcMotorEx rightMotorClose = hardwareMap.get(DcMotorEx.class, "rightMotorClose");
        DcMotorEx leftMotorFar = hardwareMap.get(DcMotorEx.class, "leftMotorFar");
        DcMotorEx leftMotorClose = hardwareMap.get(DcMotorEx.class, "leftMotorClose");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double axial = -gamepad1.left_stick_y;  // forward, back
            double lateral = gamepad1.left_stick_x; // side to side
            double yaw = gamepad1.right_stick_x; // turning

            if(gamepad1.y) {
                rightMotorClose.setPower(1);
                leftMotorClose.setPower(1);
            } else if(gamepad1.a) {
                rightMotorClose.setPower(0);
                leftMotorClose.setPower(0);
            }
        }
    }
}
