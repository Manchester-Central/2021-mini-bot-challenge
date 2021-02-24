// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class PidGyro extends CommandBase {
  private final RomiDrivetrain m_gyroDrivetrain;
  double m_targetAngle;
  private final PIDController m_pidController = new PIDController(0.2, 0.0, 0.0);

  public PidGyro(double targetAngle, RomiDrivetrain gyroDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_targetAngle = targetAngle;
    m_gyroDrivetrain = gyroDrivetrain;
    addRequirements(gyroDrivetrain);
  }

  // public getheading;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.setSetpoint(m_targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnPower = m_pidController.calculate(m_gyroDrivetrain.getAngle());
    m_gyroDrivetrain.arcadeDrive(0, Math.min(0.3,turnPower));//TO DO FiX SPEED TOO FAST
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }
}
