package org.firstinspires.ftc.teamcode;

public class MotorController {
    private static final double Kp = 0.1;  // Proportional gain
    private static final double Ki = 0.01; // Integral gain

    private static final double MAX_INTEGRAL = 10.0; // Maximum value for the integral term

    // Motor position in degrees
    private static double currentPosition = 0;

    public static void main(String[] args) {
        // Example: Hold motor at 90 degrees
        runToPosition(90);
    }

    // PID loop to hold the motor at a certain position
    public static void runToPosition(int targetPosition) {
        double error, integral = 0;

        // Run the loop for a certain number of iterations or until the error is small enough
        for (int iteration = 0; iteration < 1000; iteration++) {
            // Calculate the error
            error = targetPosition - currentPosition;

            // Update the integral term with a cap
            integral += error;
            if (integral > MAX_INTEGRAL) {
                integral = MAX_INTEGRAL;
            } else if (integral < -MAX_INTEGRAL) {
                integral = -MAX_INTEGRAL;
            }

            // Calculate the motor output using PID formula
            double output = Kp * error + Ki * integral;

            // Apply the motor output (this is a simplified example, you'd need to interface with the actual motor)
            applyMotorOutput(output);

            // Update the current position (this is a simplified example, you'd need to read from motor encoders)
            currentPosition += output;

            // Check if the motor is at the desired position
            if (Math.abs(error) < 0.1) {
                System.out.println("Motor reached the desired position.");
                break;
            }

            // Pause for a short duration (simulating time passing)
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    // Method to apply motor output (simplified example)
    private static void applyMotorOutput(double output) {
        // This is where you would interface with the actual motor to apply the output
        // (e.g., set motor power or adjust motor position)
        System.out.println("Applying motor output: " + output);
    }
}


