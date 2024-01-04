package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Intake extends LinearOpMode{

    public void runOpMode() throws InterruptedException {

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.triangle) {
                intake.setPower(0.5);
            }
            else {
                intake.setPower(0);
            }
        }
    }
}
