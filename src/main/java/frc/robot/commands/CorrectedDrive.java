// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class CorrectedDrive extends CommandBase {
  private static final double kDefaultPower = 0.5;
  RomiDrivetrain m_drivetrain;
  double m_arcLength_in;
  double m_turningRadius_in;
  boolean m_isLeft;
  double m_leftPower;
  double m_rightPower;
  double m_leftTarget_in;
  double m_rightTarget_in;
  double m_outerRadius_in;
  double m_innerRadius_in;

	long rightLastTimeUpdate = 0;
	long leftLastTimeUpdate = 0;

  /** Creates a new ArcDrive. */
  public CorrectedDrive(double turningRadius_in, double arcLength_in, boolean isLeft, RomiDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_arcLength_in = arcLength_in;
    m_turningRadius_in = turningRadius_in;
    m_isLeft = isLeft;
    double romiRadius = RomiDrivetrain.kTrackWidthInch / 2;
    m_outerRadius_in = m_turningRadius_in + romiRadius;
    m_innerRadius_in = m_turningRadius_in - romiRadius;
    addRequirements(drivetrain);
  }

  /** Creates a new ArcDrive. */
  public CorrectedDrive(double distance_in, RomiDrivetrain drivetrain) {
    this(-1, distance_in, false, drivetrain);
    m_outerRadius_in = -1;
    m_innerRadius_in = -1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
    leftLastTimeUpdate = 0;
    rightLastTimeUpdate = 0;
    if(m_isLeft) {
      m_leftPower = kDefaultPower * m_innerRadius_in / m_outerRadius_in ;
      m_rightPower = kDefaultPower; 
      m_leftTarget_in = m_arcLength_in * (m_innerRadius_in / m_turningRadius_in);
      m_rightTarget_in = m_arcLength_in * (m_outerRadius_in / m_turningRadius_in);
    } else{
      m_leftPower = kDefaultPower;
      m_rightPower = kDefaultPower * m_innerRadius_in / m_outerRadius_in;
      m_leftTarget_in = m_arcLength_in * (m_outerRadius_in / m_turningRadius_in);
      m_rightTarget_in = m_arcLength_in * (m_innerRadius_in / m_turningRadius_in);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double arcTraveled_in = getArcTraveledInch();

    double leftError_in = m_leftTarget_in - m_drivetrain.getLeftDistanceInch();
    double leftErrorScaled = leftError_in / m_leftTarget_in;

    double rightError_in = m_rightTarget_in - m_drivetrain.getRightDistanceInch();
    double rightErrorScaled = rightError_in / m_rightTarget_in;

    double turnError = (leftErrorScaled - rightErrorScaled) * arcTraveled_in;

    double forwardGain = 0.09;
    double turnGain = 0.3;
    double minSpeed = 0.1;
    double maxSpeed = 0.3;
    double maxSpeedChange = 0.015;

		double leftOutput  = (leftError_in  * forwardGain) + (turnError * turnGain);
		double rightOutput = (rightError_in * forwardGain) - (turnError * turnGain);
		
		double absLeftOutput  = Math.abs(leftOutput);
    double absRightOutput = Math.abs(rightOutput);
    
    double speedDenominator = (absRightOutput > absLeftOutput) ? absRightOutput : absLeftOutput;
		
		if (speedDenominator != 0) {
			if (absLeftOutput < minSpeed || absRightOutput < minSpeed) {
				rightOutput *= (minSpeed / speedDenominator);
				leftOutput  *= (minSpeed / speedDenominator);
		
			
			} else if (absRightOutput > maxSpeed || absLeftOutput > maxSpeed) {
				rightOutput *= (maxSpeed / speedDenominator);
				leftOutput  *= (maxSpeed / speedDenominator);
			}
    }
    
		double absLeftTarget = Math.abs(m_leftTarget_in);
    double absRightTarget = Math.abs(m_rightTarget_in);
    
    double biggerTarget = (absLeftTarget > absRightTarget) ? absLeftTarget : absRightTarget;

		m_leftPower = throttleAcceleration(m_leftPower,  leftOutput, maxSpeedChange * (absLeftTarget / biggerTarget), 1, "left");
    m_rightPower = throttleAcceleration(m_rightPower, rightOutput, maxSpeedChange * (absRightTarget / biggerTarget), 1, "right");

    System.out.println(leftErrorScaled + " " + rightErrorScaled + " " + turnError + " " + m_leftPower + " " + m_rightPower);
    m_drivetrain.TankDrive(m_leftPower, m_rightPower);
  }

  private double throttleAcceleration (double currentSet, double proportionalSet, double maxAcceleration, long changeRate, String direction) {
		
		boolean proportionalSetIsPositive = proportionalSet > 0.0;
		
		double velocityChange = (proportionalSetIsPositive) ? proportionalSet - currentSet : Math.abs(currentSet - proportionalSet);
		
		if (currentSet == 0.0) {
			velocityChange = Math.abs(currentSet - proportionalSet); 
		} else if (proportionalSet == 0.0) {
			velocityChange = -Math.abs(currentSet - proportionalSet); 
		}
		
		// if ΔV > max acceleration
		if (velocityChange > maxAcceleration) {
			
			if ((proportionalSet > 0 && currentSet < 0) || (proportionalSet < 0 && currentSet > 0)) {
				
				proportionalSet = 0.0;
				
			} else if (rightLastTimeUpdate + changeRate < System.currentTimeMillis() && direction.equals("right")) {
				
				double addAcceleration = (proportionalSetIsPositive) ? maxAcceleration : -maxAcceleration;
				
				proportionalSet = currentSet + addAcceleration;
				
				rightLastTimeUpdate = System.currentTimeMillis();
			
			}  else if (leftLastTimeUpdate + changeRate < System.currentTimeMillis() && direction.equals("left")) {
				
				double addAcceleration = (proportionalSetIsPositive) ? maxAcceleration : -maxAcceleration;
				
				proportionalSet = currentSet + addAcceleration;
				
				leftLastTimeUpdate = System.currentTimeMillis();
			
			} else {
				
				proportionalSet = currentSet;
				
			}
			
		}
		
		return proportionalSet;
		
	}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getArcTraveledInch()  > m_arcLength_in;
  }

  private double getArcTraveledInch() {
    double leftDistance_in = m_drivetrain.getLeftDistanceInch();
    double rightDistance_in = m_drivetrain.getRightDistanceInch();
    double middleDistance_in = (leftDistance_in + rightDistance_in) / 2;
    return middleDistance_in;
  }
}
