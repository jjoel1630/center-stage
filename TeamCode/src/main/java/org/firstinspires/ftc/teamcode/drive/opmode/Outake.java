package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.message.redux.ReceiveGamepadState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class Outake extends LinearOpMode {
    // claw min and max
    public static double CLAW_MIN = 0;
    public static double CLAW_MAX = 1;
    private static ReceiveGamepadState.Gamepad gamepad2;

    public Servo s = hardwareMap.servo.get("claw");
    public static double servoPos = CLAW_MIN;



    public static void openBothServos(){
        // if right control is pressed then servo 1 is realised
        if(gamepad2.right_bumper == 1.0){
            servoPos = CLAW_MIN;
        }
        // if left control is pressed then servo 2 is realised
        else if(gamepad2.left_bumper == 1.0){
            servoPos = CLAW_MAX;
        }
        else if (gamepad2.y - 1.0) {
            servoPos = CLAW_MAX
        }
            s.setPosition(servoPos);
    }




    }
}