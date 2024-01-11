package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class ServoMotor extends LinearOpMode {
    public Servo servo;

    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");

        while(opModeIsActive()) {
            if(gamepad1.dpad_up) {
                servo.setPosition(1);
            }
            else if(gamepad1.dpad_down) {
                servo.setPosition(0);
            }
        }
    }
}
