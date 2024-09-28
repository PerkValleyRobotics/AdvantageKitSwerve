package frc.robot.subsystems.Launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class LauncherIOSim implements LauncherIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    
    private DCMotorSim leftSim = new DCMotorSim(DCMotor.getNeo550(1), 0.33,  0.000298);
    private DCMotorSim rightSim = new DCMotorSim(DCMotor.getNeo550(1), 0.33,  0.000298);

    double leftAppliedVolts = 0.0;
    double rightAppliedVolts = 0.0;

    @Override
    public void updateInputs(LauncherIOInputs inputs) {
        leftSim.update(LOOP_PERIOD_SECS);
        rightSim.update(LOOP_PERIOD_SECS);

        inputs.leftWheelVelocityRadPerSec = leftSim.getAngularVelocityRadPerSec();
        inputs.leftWheelAppliedVolts = leftAppliedVolts;
        inputs.leftWheelCurrentAmps = new double[] {Math.abs(leftSim.getCurrentDrawAmps())};

        inputs.rightWheelVelocityRadPerSec = rightSim.getAngularVelocityRadPerSec();
        inputs.rightWheelAppliedVolts = rightAppliedVolts;
        inputs.rightWheelCurrentAmps = new double[] {Math.abs(rightSim.getCurrentDrawAmps())};
    }

    @Override
    public void setLeftVoltage(double volts) {
        leftAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        leftSim.setInputVoltage(leftAppliedVolts);
    }

    @Override
    public void setRightVoltage(double volts) {
        rightAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        rightSim.setInputVoltage(rightAppliedVolts);
    }
}
