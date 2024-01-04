package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Gyroscope;


public abstract class Arm extends LinearOpMode {
    DcMotor arm;

    public void runOpMode() throws InterruptedException {
        int armUpPosition = 800;
        int armDownPosition = 0;

        double position = arm.getCurrentPosition();
        double desiredPosition = arm.getTargetPosition();

        arm = hardwareMap.dcMotor.get("Arm");

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.circle && position < armUpPosition) {
                arm.setTargetPosition(armUpPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.2);
            }
            else if (gamepad2.square && position > armDownPosition) {
                arm.setTargetPosition(armDownPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(-0.2);
            }
            else {
               arm.setPower(0);
            }

            telemetry.addData("Encoder Position", position);
            telemetry.addData("Desired Position", desiredPosition);
            telemetry.update();
        }
    }
}
