// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class PidDrive extends CommandBase {
  /** Creates a new PidDrive. */
  private double m_distanceTargetLeft_in;
  private double m_distanceTargetRight_in;
  private RomiDrivetrain m_drivetrain;

  public PidDrive(double distanceTargetRight_in, double distanceTargetLeft_in, RomiDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_distanceTargetLeft_in = distanceTargetLeft_in;
    m_distanceTargetRight_in = distanceTargetRight_in;
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.SetPidTarget(m_drivetrain.getLeftDistanceInch() + m_distanceTargetLeft_in,
        m_drivetrain.getRightDistanceInch() + m_distanceTargetRight_in);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.PidDrive();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.TargetReached();
  }
}
