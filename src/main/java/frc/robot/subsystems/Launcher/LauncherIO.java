package frc.robot.subsystems.Launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
    @AutoLog
    public static class LauncherIOInputs {
        public double leftWheelVelocityRadPerSec = 0.0;
        public double leftWheelAppliedVolts = 0.0;
        public double[] leftWheelCurrentAmps = new double[] {};

        public double rightWheelVelocityRadPerSec = 0.0;
        public double rightWheelAppliedVolts = 0.0;
        public double[] rightWheelCurrentAmps = new double[] {};
    }

    // Update the set of loggable inputs
    public default void updateInputs(LauncherIOInputs inputs) {}

    // Run the left motor at a specified voltage 
    public default void setLeftVoltage(double volts) {}

    // Run the right motor at a specified voltage 
    public default void setRightVoltage(double volts) {}
}
