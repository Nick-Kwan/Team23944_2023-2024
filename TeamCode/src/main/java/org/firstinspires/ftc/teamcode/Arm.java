package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Arm extends LinearOpMode {
    DcMotor arm;

    public void runOpMode() throws InterruptedException {
        int armUpPosition = 1000;
        int armDownPosition = 0;

        arm = hardwareMap.dcMotor.get("Arm");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(armDownPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.cross) {
                arm.setTargetPosition(armDownPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.5);
            }

            double position = arm.getCurrentPosition();
            double desiredPosition = arm.getTargetPosition();

            telemetry.addData("Encoder Position", position);
            telemetry.addData("Desired Position", desiredPosition);
            telemetry.update();
        }
    }
}
