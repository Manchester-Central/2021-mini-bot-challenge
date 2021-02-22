// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class PidTurn extends CommandBase {
  RomiDrivetrain m_romiDrivetrain;
  double m_TargetDegrees;
  /** Creates a new PidTurn. */
  public PidTurn(double TargetDegrees, RomiDrivetrain romiDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_romiDrivetrain = romiDrivetrain;
    m_TargetDegrees = TargetDegrees;
    addRequirements(romiDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("initialize");
    m_romiDrivetrain.SetPidTargetAngle(m_TargetDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("execute");
    m_romiDrivetrain.PidDrive();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("end -interrupted? " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("isFinished");
    return m_romiDrivetrain.TargetReached();
  }
}
