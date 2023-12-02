package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "FTCAutonomousRightTurn (Java)")

public abstract class FTCAutonomousRightTurn extends LinearOpMode
{
    private DcMotor RightDrive;
    private DcMotor LeftDrive;
    private DcMotor Arm;
    private DcMotor Intake;

    ElapsedTime runTime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.7;
    static final double TURN_SPEED = 0.5;

    public void runOpMode() {
        LeftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        RightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        LeftDrive.setDirection(DcMotor.Direction.REVERSE);
        RightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        LeftDrive.setPower(FORWARD_SPEED);
        RightDrive.setPower(FORWARD_SPEED);
        runTime.reset();

        while (opModeIsActive() && (runTime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runTime.seconds());
            telemetry.update();


            LeftDrive.setPower(TURN_SPEED);
            RightDrive.setPower(-TURN_SPEED);
            runTime.reset();

            if (opModeIsActive() && (runTime.seconds()) < 1.3) {
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runTime.seconds());
            telemetry.update(); }


            LeftDrive.setPower(-FORWARD_SPEED);
            RightDrive.setPower(-FORWARD_SPEED);
            runTime.reset();

            else if ((opModeIsActive() && ((runTime.seconds()) < 1.0)) || ((runTime.seconds()) < 2.0)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runTime.seconds());
            telemetry.update(); }


            LeftDrive.setPower(0);
            RightDrive.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }
    }
}
