package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {


    private double integralSum = 0;
    private final double kp;
    private final double ki;
    private final double kd;
    private double kf;
    private double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    public PID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
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
