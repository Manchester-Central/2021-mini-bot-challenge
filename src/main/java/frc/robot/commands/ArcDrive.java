// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class ArcDrive extends CommandBase {
  private static final double kDefaultPower = 0.5;
  RomiDrivetrain m_drivetrain;
  double m_arcLength_in;
  double m_leftPower;
  double m_rightPower;
  /** Creates a new ArcDrive. */
  public ArcDrive(double turningRadius_in, double arcLength_in, boolean isLeft, RomiDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_arcLength_in = arcLength_in;
    addRequirements(drivetrain);

    double romiRadius = RomiDrivetrain.kTrackWidthInch / 2;
    double outerRadius = turningRadius_in + romiRadius;
    double innerRadius = turningRadius_in - romiRadius;
    
    if(isLeft) {
      m_leftPower = kDefaultPower * innerRadius / outerRadius ;
      m_rightPower = kDefaultPower; 
    } else{
      m_leftPower = kDefaultPower;
      m_rightPower = kDefaultPower * innerRadius / outerRadius;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.TankDrive(m_leftPower, m_rightPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double leftDistance_in = m_drivetrain.getLeftDistanceInch();
    double rightDistance_in = m_drivetrain.getRightDistanceInch();
    double middleDistance_in = (leftDistance_in + rightDistance_in) / 2;
    return middleDistance_in > m_arcLength_in;
  }
}
