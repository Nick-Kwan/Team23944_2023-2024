package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "FTCAutonomousRightTurn (Java)")

public abstract class FTCAutonomousRightTurn extends LinearOpMode
{
    private DcMotor Arm;
    private DcMotor Intake;

    ElapsedTime runTime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.7;
    static final double TURN_SPEED = 0.5;

    public void runOpMode() {
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        leftDrive.setPower(FORWARD_SPEED);
        rightDrive.setPower(FORWARD_SPEED);
        runTime.reset();

        while (opModeIsActive() && (runTime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runTime.seconds());
            telemetry.update(); }


            leftDrive.setPower(TURN_SPEED);
            rightDrive.setPower(-TURN_SPEED);
            runTime.reset();

            while (opModeIsActive() && (runTime.seconds()) < 1.3) {
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runTime.seconds());
                telemetry.update(); }


            leftDrive.setPower(-FORWARD_SPEED);
            rightDrive.setPower(-FORWARD_SPEED);
            runTime.reset();

             while ((opModeIsActive() && ((runTime.seconds()) < 1.0)) || ((runTime.seconds()) < 2.0)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runTime.seconds());
                telemetry.update(); }


             leftDrive.setPower(0);
             rightDrive.setPower(0);

             telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }
    }
