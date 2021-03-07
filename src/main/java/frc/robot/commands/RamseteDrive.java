// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.RomiDrivetrain;

public class RamseteDrive extends SequentialCommandGroup {

  /** Creates a new RamseteDrive. */
  public RamseteDrive(RomiDrivetrain romiDrivetrain, String pathName) {
    addRequirements(romiDrivetrain);
    
    String trajectoryJSON = "paths/output/" + pathName + ".wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory" + trajectoryJSON, ex.getStackTrace());
      return;
    }

    final Pose2d initialPose = trajectory.getInitialPose();
    var startCommand = new InstantCommand(() -> romiDrivetrain.resetOdometry(initialPose), romiDrivetrain);
    var driveCommand = new RamseteCommand(
      trajectory,
      romiDrivetrain::getPose2d,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
      new DifferentialDriveKinematics(RomiDrivetrain.kTrackWidthMeters),
      romiDrivetrain::getWheelSpeeds,
      new PIDController(Constants.kRamseteP, Constants.kRamseteI, Constants.kRamseteD),
      new PIDController(Constants.kRamseteP, Constants.kRamseteI, Constants.kRamseteD),
      romiDrivetrain::tankDriveVolts,
      romiDrivetrain);
    var endCommand = new InstantCommand(() -> romiDrivetrain.tankDriveVolts(0.0, 0.0), romiDrivetrain);

    addCommands(startCommand, driveCommand, endCommand);
  }
}
