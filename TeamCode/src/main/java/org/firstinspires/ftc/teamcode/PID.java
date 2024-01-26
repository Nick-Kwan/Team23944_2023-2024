package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID extends LinearOpMode {

    DcMotorEx motor;

    double integralSum = 0;
    double kp = 1;
    double ki = 0;
    double kd = 0;
    double kf = 1;
    private double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "PIDMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while(opModeIsActive()) {
            double power = PIDControl(1000, motor.getVelocity());
            motor.setPower(power);
        }
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error*timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return (error * kp) + (derivative * kd) + (integralSum * ki) + (reference * kf);
    }
}
