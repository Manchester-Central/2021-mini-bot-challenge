// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class Unload extends ParallelDeadlineGroup {

  /** Creates a new Unload command. */
  public Unload(Intake intake, double timeout) {
    super(new WaitCommand(timeout), new RunCommand(() -> intake.setPower(-0.5), intake));
  }
}
