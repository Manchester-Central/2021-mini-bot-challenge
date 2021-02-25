// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.data.Direction;
import frc.robot.subsystems.RomiDrivetrain;

public class ArcDriveVelocity extends CommandBase {

  private double kDefaultSpeed = 12;
  private RomiDrivetrain m_drivetrain;
  private double m_arcLength_in;
  private double m_leftTargetSpeed;
  private double m_rightTargetSpeed;
  private double m_leftPower = 0;
  private double m_rightPower = 0;

  private PIDController m_pidVelocityLeft = new PIDController(0.0025, 0.001, 0);
  private PIDController m_pidVelocityRight = new PIDController(0.0025, 0.001, 0);

  /** Creates a new DistanceAutoDrive. */
  public ArcDriveVelocity(double turningRadius, double arcLength, Direction direction, RomiDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drivetrain = drivetrain;
    m_arcLength_in = arcLength;
    double romiRadius = RomiDrivetrain.kTrackWidthInch / 2;
    double outerRadius = (turningRadius + romiRadius);
    double innerRadius = (turningRadius - romiRadius);
    if (direction == Direction.Left) {
      m_leftTargetSpeed = (innerRadius / outerRadius) * kDefaultSpeed;
      m_rightTargetSpeed = kDefaultSpeed;
    } else if(direction == Direction.Right) {
      m_leftTargetSpeed = kDefaultSpeed;
      m_rightTargetSpeed = (innerRadius / outerRadius) * kDefaultSpeed;
    } else {
      m_leftTargetSpeed = kDefaultSpeed;
      m_rightTargetSpeed = kDefaultSpeed;
    }
    m_pidVelocityLeft.setSetpoint(m_leftTargetSpeed);
    m_pidVelocityRight.setSetpoint(m_rightTargetSpeed);
    addRequirements(drivetrain);
  }

  public ArcDriveVelocity(double length, RomiDrivetrain drivetrain) {
    this(-1, length, Direction.Straight, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_leftPower = m_leftPower + m_pidVelocityLeft.calculate(m_drivetrain.getLeftSpeedInchesPerSec());
    m_rightPower = m_rightPower + m_pidVelocityRight.calculate(m_drivetrain.getRightSpeedInchesPerSec());
    m_drivetrain.TankDrive(m_leftPower, m_rightPower /* * 0.96 */);
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
