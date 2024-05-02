package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    private final DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
    private double power = 0;

    ElapsedTime time = new ElapsedTime();

    public Intake(double power) {
        this.power = power;
    }

    public double intakeControl() {

        power = -0.55;
        if (gamepad2.triangle) {
            intake.setPower(power);
        }
        power = 0;
        if(gamepad2.cross){
            intake.setPower(power);
        }
        return power;
    }
}
