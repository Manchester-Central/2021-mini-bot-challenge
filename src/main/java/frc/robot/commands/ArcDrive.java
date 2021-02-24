// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class ArcDrive extends CommandBase {

  public enum Direction {
    Left, Right
  }

  private double kDefaultSpeed = 0.5;
  private RomiDrivetrain m_drivetrain;
  private double m_arcLength_in;
  private double m_leftPower;
  private double m_rightPower;

  /** Creates a new DistanceAutoDrive. */
  public ArcDrive(double turningRadius, double arcLength, Direction direction, RomiDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drivetrain = drivetrain;
    m_arcLength_in = arcLength;
    double romiRadius = RomiDrivetrain.kTrackWidthInch / 2;
    double outerRadius = (turningRadius + romiRadius);
    double innerRadius = (turningRadius - romiRadius);
    if (direction == Direction.Left) {
      m_leftPower = (innerRadius / outerRadius) * kDefaultSpeed;
      m_rightPower = kDefaultSpeed;
    } else {
      m_leftPower = kDefaultSpeed;
      m_rightPower = (innerRadius / outerRadius) * kDefaultSpeed;
    }
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.TankDrive(m_leftPower, m_rightPower * 0.96);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.TankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double leftDistance = m_drivetrain.getLeftDistanceInch();
    double rightDistance = m_drivetrain.getRightDistanceInch();
    double middleDistance = (leftDistance + rightDistance) / 2;
    return middleDistance > m_arcLength_in;
  }
}
