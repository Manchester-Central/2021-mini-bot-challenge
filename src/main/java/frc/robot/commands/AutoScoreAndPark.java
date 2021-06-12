// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RomiDrivetrain;

public class AutoScoreAndPark extends SequentialCommandGroup {
  /** Creates a new AutoScoreAndPark. */
  public AutoScoreAndPark (Command firstCommand, Intake intake, RomiDrivetrain drivetrain){
    addCommands(
      // Run the course, with intake on
      new ParallelDeadlineGroup(
        firstCommand, // will interrupt the intake command when done
        new RunIntake(intake, true, true)
      ),
      // Unload the golf balls for 2 seconds
      new Unload(intake, 2),
      // Park
      new PathDrive("AllianceAnticsBluePark", drivetrain)
    );
  }
  public AutoScoreAndPark(String pathName, Intake intake, RomiDrivetrain drivetrain) {
    this(new PathDrive(pathName, drivetrain), intake, drivetrain);
  }
}
