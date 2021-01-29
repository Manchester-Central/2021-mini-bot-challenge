// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleLED extends CommandBase {
  /** Creates a new Toggleled. */
  private DigitalOutput r_led;

  public ToggleLED(DigitalOutput digitalout) {
    // Use addRequirements() here to declare subsystem dependencies.
    r_led = digitalout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    r_led.set(!r_led.get());
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
    return true;
  }
}