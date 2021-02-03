// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.rmi.server.RMIClassLoader;
import java.sql.Time;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class TimedAutoDrive extends CommandBase {
  double m_time_sec;
  RomiDrivetrain m_drivetrain;
  double m_powerLeft;
  double m_powerRight;
  double m_Start_sec;
  /** Creates a new TimedAutoDrive. */
  public TimedAutoDrive(double time_sec, RomiDrivetrain drivetrain, double powerLeft, double powerRight) {

    // Use addRequirements() here to declare subsystem dependencies.

    m_time_sec = time_sec;
    m_drivetrain = drivetrain;
    m_powerLeft = powerLeft;
    m_powerRight = powerRight; 
    m_Start_sec = RobotController.getFPGATime();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
