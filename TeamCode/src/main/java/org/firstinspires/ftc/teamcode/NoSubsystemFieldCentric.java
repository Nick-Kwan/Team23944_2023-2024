package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="VeryBasicFieldCentric", group="DriveModes")
public class NoSubsystemFieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"frontLeft");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"backLeft");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"frontRight");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"backRight");
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class,"arm");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class,"intake");
        Servo servo = hardwareMap.servo.get("servoMotor");
        PID pidMotor = new PID();

        int armUpPosition = 200;
        int armDownPosition = 0;

        int armPosition = arm.getCurrentPosition();
        int desiredArmPosition = armUpPosition;

        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            pidMotor.runOpMode();
            pidMotor.PIDControl(desiredArmPosition, armPosition);
            arm.setTargetPosition(desiredArmPosition);

            if(gamepad2.triangle) {
                intake.setPower(-0.55);
            }
            if(gamepad2.cross) {
                intake.setPower(0);
            }
            if (gamepad2.circle) {
                arm.setTargetPosition(desiredArmPosition);
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                arm.setPower(0.3);
            }
            if (gamepad2.square) {
                arm.setTargetPosition(armDownPosition);
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                arm.setPower(-0.3);
            }
            if(gamepad2.dpad_up) {
                servo.setPosition(0);
            }
            if(gamepad2.dpad_down) {
                servo.setPosition(1);
            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.dpad_up) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bots rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            //rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}