// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class CorrectedDrive extends CommandBase {
  private static final double kForwardGain = 0.1;
  private static final double kTurnGain = 0.4;
  private static final double kMinPower = 0.05;
  private static final double kMaxPower = 0.3;
  private static final double kMaxPowerChange = 0.015;
  private static final double kChangeRate_ms = 1;
  private static final double kTolerance_in = 0.1;

  RomiDrivetrain m_drivetrain;
  double m_arcLength_in;
  double m_leftPower;
  double m_rightPower;
  double m_leftTarget_in;
  double m_rightTarget_in;

  long rightLastTimeUpdate = 0;
  long leftLastTimeUpdate = 0;

  /** Creates a Corrected Drive for turning on an arc */
  public CorrectedDrive(double turningRadius_in, double arcLength_in, boolean isLeft, RomiDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    double romiRadius = RomiDrivetrain.kTrackWidthInch / 2;
    double outerRadius_in = turningRadius_in + romiRadius;
    double innerRadius_in = turningRadius_in - romiRadius;
    double leftTarget_in, rightTarget_in;

    if (isLeft) {
      leftTarget_in = arcLength_in * (innerRadius_in / turningRadius_in);
      rightTarget_in = arcLength_in * (outerRadius_in / turningRadius_in);
    } else {
      leftTarget_in = arcLength_in * (outerRadius_in / turningRadius_in);
      rightTarget_in = arcLength_in * (innerRadius_in / turningRadius_in);
    }

    setMembers(arcLength_in, leftTarget_in, rightTarget_in, drivetrain);
  }

  /** Creates a Corrected Drive for going straight */
  public CorrectedDrive(double distance_in, RomiDrivetrain drivetrain) {
    setMembers(distance_in, distance_in, distance_in, drivetrain);
  }

  /** Sets the needed members of the class */
  private void setMembers(double middleTarget_in, double leftTarget_in, double rightTarget_in, RomiDrivetrain drivetrain) {
    m_arcLength_in = middleTarget_in;
    m_leftTarget_in = leftTarget_in;
    m_rightTarget_in = rightTarget_in;
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
    leftLastTimeUpdate = 0;
    rightLastTimeUpdate = 0;
    m_leftPower = 0;
    m_rightPower = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the travelled distance of the middle of the robot
    double arcTraveled_in = getArcTraveledInch();

    // Calculate the error and error % from the target
    double leftError_in = m_leftTarget_in - m_drivetrain.getLeftDistanceInch();
    double leftErrorScaled = leftError_in / m_leftTarget_in;
    double rightError_in = m_rightTarget_in - m_drivetrain.getRightDistanceInch();
    double rightErrorScaled = rightError_in / m_rightTarget_in;

    // Determine how far the left or right if off from the given arc (or straight
    // line)
    double turnError = (leftErrorScaled - rightErrorScaled) * m_arcLength_in;

    // Outputs are determined by how far from the target and the turn error
    // (multiplied by gains)
    double leftOutput = (leftError_in * kForwardGain) + (turnError * kTurnGain);
    double rightOutput = (rightError_in * kForwardGain) - (turnError * kTurnGain);

    // Get the absolute value of the outputs
    double absLeftOutput = Math.abs(leftOutput);
    double absRightOutput = Math.abs(rightOutput);

    // Find the larger (magnitude) of the outputs
    double biggerOutput = (absRightOutput > absLeftOutput) ? absRightOutput : absLeftOutput;
    double smallerOutput = (absRightOutput < absLeftOutput) ? absRightOutput : absLeftOutput;

    if (biggerOutput != 0) {
      if (absRightOutput < kMinPower || absLeftOutput < kMinPower) {
        // If output it too low to move, proportionally adjust both speeds
        leftOutput *= (kMinPower / smallerOutput);
        rightOutput *= (kMinPower / smallerOutput);
      } else if (absLeftOutput > kMaxPower || absRightOutput > kMaxPower) {
        // If output it too high, proportionally adjust both speeds
        leftOutput *= (kMaxPower / biggerOutput);
        rightOutput *= (kMaxPower / biggerOutput);
      }
    }

    // Get the larger target distance
    double absLeftTarget_in = Math.abs(m_leftTarget_in);
    double absRightTarget_in = Math.abs(m_rightTarget_in);
    double biggerTarget_in = (absLeftTarget_in > absRightTarget_in) ? absLeftTarget_in : absRightTarget_in;

    // the max acceleration is based propoprtionally on which side needs to travel further
    double leftMaxAcceleration = kMaxPowerChange * (absLeftTarget_in / biggerTarget_in);
    double rightMaxAcceleration = kMaxPowerChange * (absRightTarget_in / biggerTarget_in);

    // Set the speeds, but throttle velocity change over time
    m_leftPower = throttleAcceleration(m_leftPower, leftOutput, leftMaxAcceleration, true);
    m_rightPower = throttleAcceleration(m_rightPower, rightOutput, rightMaxAcceleration, false);
    m_drivetrain.TankDrive(m_leftPower, m_rightPower);
  }

  private double throttleAcceleration(double currentPower, double proposedPower, double maxAcceleration, boolean isLeft) {

    boolean proposedPowerIsPositive = proposedPower > 0.0;

    // Figure out the power change with respect to 0
    double powerChange = (proposedPowerIsPositive) 
      ? proposedPower - currentPower
      : Math.abs(currentPower - proposedPower);

    // Handle 0 cases
    if (currentPower == 0.0) {
      powerChange = Math.abs(proposedPower);
    } else if (proposedPower == 0.0) {
      powerChange = -Math.abs(currentPower);
    }

    // if ΔV > max acceleration
    if (powerChange > maxAcceleration) {
      if ((proposedPower > 0 && currentPower < 0) || (proposedPower < 0 && currentPower > 0)) {
        // If current and proposed are opposite directions, set power to 0 first
        proposedPower = 0.0;
      } 
      else if (rightLastTimeUpdate + kChangeRate_ms < System.currentTimeMillis()) {
        // if enough have time has passed between updates, add acceleration
        double addAcceleration = (powerChange > 0) ? maxAcceleration : -maxAcceleration;
        proposedPower = currentPower + addAcceleration;
        if(isLeft) {
          leftLastTimeUpdate = System.currentTimeMillis();
        } else {
          rightLastTimeUpdate = System.currentTimeMillis();
        }
      } else {
        // If not enough time has passed between updates, use current power
        proposedPower = currentPower;
      }
    }

    return proposedPower;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getArcTraveledInch() > m_arcLength_in - kTolerance_in;
  }

  private double getArcTraveledInch() {
    double leftDistance_in = m_drivetrain.getLeftDistanceInch();
    double rightDistance_in = m_drivetrain.getRightDistanceInch();
    double middleDistance_in = (leftDistance_in + rightDistance_in) / 2;
    return middleDistance_in;
  }
}
