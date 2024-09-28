// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
  private final LauncherIO io;
  private final LauncherIOInputsAutoLogged laucherInputs = new LauncherIOInputsAutoLogged();

  private final PIDController leftFeedBack;
  private final PIDController rightFeedBack;
  private SimpleMotorFeedforward leftFeedForward;
  private SimpleMotorFeedforward rightFeedForward;

  private Double leftSetpoint = null; // Set to null to stop feedback controll
  private Double rightSetpoint = null;

  /** Creates a new Launcher. */
  public Launcher(LauncherIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        leftFeedForward = new SimpleMotorFeedforward(0, 0);
        leftFeedBack = new PIDController(0, 0, 0);
        rightFeedForward = new SimpleMotorFeedforward(0, 0);
        rightFeedBack = new PIDController(0, 0, 0);
        break;
      case SIM:
        leftFeedForward = new SimpleMotorFeedforward(0, 0);
        leftFeedBack = new PIDController(0, 0, 0);
        rightFeedForward = new SimpleMotorFeedforward(0, 0);
        rightFeedBack = new PIDController(0, 0, 0);
        break;
    
      default:
        leftFeedForward = new SimpleMotorFeedforward(0, 0);
        leftFeedBack = new PIDController(0, 0, 0);
        rightFeedForward = new SimpleMotorFeedforward(0, 0);
        rightFeedBack = new PIDController(0, 0, 0);
        break;
    }
    SmartDashboard.putNumber("Left_Launcher_P", leftFeedBack.getP());
    SmartDashboard.putNumber("Left_Launcher_I", leftFeedBack.getI());
    SmartDashboard.putNumber("Left_Launcher_D", leftFeedBack.getD());
    SmartDashboard.putNumber("Left_Launcher_S", leftFeedForward.ks);
    SmartDashboard.putNumber("Left_Launcher_V", leftFeedForward.kv);

    SmartDashboard.putNumber("Right_Launcher_P", rightFeedBack.getP());
    SmartDashboard.putNumber("Right_Launcher_I", rightFeedBack.getI());
    SmartDashboard.putNumber("Right_Launcher_D", rightFeedBack.getD());
    SmartDashboard.putNumber("Right_Launcher_S", rightFeedForward.ks);
    SmartDashboard.putNumber("Right_Launcher_V", rightFeedForward.kv);
  }

  @Override
  public void periodic() {
    io.updateInputs(laucherInputs);
    Logger.processInputs("Launcher", laucherInputs);

    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Launcher/LeftSetpointRPM", 0.0);
      Logger.recordOutput("Launcher/RightSetpointRPM", 0.0);
    }

    // Update Pid with values from shuffleboard
    leftFeedBack.setP(SmartDashboard.getNumber("Left_Launcher_P", 0));
    leftFeedBack.setI(SmartDashboard.getNumber("Left_Launcher_I", 0));
    leftFeedBack.setD(SmartDashboard.getNumber("Left_Launcher_D", 0));
    leftFeedForward = new SimpleMotorFeedforward(SmartDashboard.getNumber("Left_Launcher_S", 0), SmartDashboard.getNumber("Left_Launcher_V", 0));

    rightFeedBack.setP(SmartDashboard.getNumber("Right_Launcher_P", 0));
    rightFeedBack.setI(SmartDashboard.getNumber("Right_Launcher_I", 0));
    rightFeedBack.setD(SmartDashboard.getNumber("Right_Launcher_D", 0));
    rightFeedForward = new SimpleMotorFeedforward(SmartDashboard.getNumber("Right_Launcher_S", 0), SmartDashboard.getNumber("Right_Launcher_V", 0));

    // Run flywheel feedback
    if (!(leftSetpoint == null || rightSetpoint == null))  {

      double leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(leftSetpoint);
      io.setLeftVoltage(
          // leftFeedForward.calculate(leftVelocityRadPerSec) 
             leftFeedBack.calculate(laucherInputs.leftWheelVelocityRadPerSec, leftVelocityRadPerSec));
      
      double rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rightSetpoint);
      io.setRightVoltage(
          // rightFeedForward.calculate(rightVelocityRadPerSec) 
             rightFeedBack.calculate(laucherInputs.rightWheelVelocityRadPerSec, rightVelocityRadPerSec));

      Logger.recordOutput("Launcher/LeftSetpointRadPerSec", leftVelocityRadPerSec);
      Logger.recordOutput("Launcher/RightSetpointRadPerSec", rightVelocityRadPerSec);
    } else {

      Logger.recordOutput("Launcher/LeftSetpointRadPerSec", 0.0);
      Logger.recordOutput("Launcher/RightSetpointRadPerSec", 0.0);
    }
  }

  public void runRPM(double rpm) {
    leftSetpoint = -rpm;
    rightSetpoint = rpm;

    Logger.recordOutput("Launcher/LeftSetpointRPM", leftSetpoint);
    Logger.recordOutput("Launcher/RightSetpointRPM", rightSetpoint);
  }

  public void stop() {
    runRPM(0);
    io.setLeftVoltage(0);
    io.setRightVoltage(0);

    System.out.println("stopping" + Logger.getRealTimestamp());

    leftSetpoint = null;
    rightSetpoint = null;
  }
}
