// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.gamepads.Gamepad;
import frc.robot.subsystems.RomiDrivetrain;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  Gamepad m_Driver;
  RomiDrivetrain m_Drivetrain;

  public ArcadeDrive(Gamepad Driver, RomiDrivetrain Drivetrain) {
    m_Driver = Driver;
    m_Drivetrain = Drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Drivetrain.arcadeDrive(m_Driver.getLeftY()*0.7, m_Driver.getRightX()*0.7, true);
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
