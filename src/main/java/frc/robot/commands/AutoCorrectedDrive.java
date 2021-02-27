// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class AutoCorrectedDrive extends CommandBase {
  private double m_middleTarget_in;
  private double m_leftTarget_in;
  private double m_rightTarget_in;
  private double m_leftPower, m_rightPower;

  private final double kForwardGain = 0.13;
  private final double kTurnGain = 0.2;
  private final double kMinPower = 0.1;
  private final double kMaxPower = 1.0;
  private final double kMaxPowerChange = 0.009;
  private final double kTolerance_in = 0.05;


  private RomiDrivetrain m_drivetrain;

  /** Creates a new AutoCorrectedDrive. */
  public AutoCorrectedDrive(double distance_in, RomiDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    setMembers(distance_in, distance_in, distance_in, drivetrain);
  }

  public AutoCorrectedDrive(double turningRadius_in, double arcLength_in, boolean isLeft, RomiDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    double romiRadius = RomiDrivetrain.kTrackWidthInch / 2;
    double outerRadius = turningRadius_in + romiRadius;
    double innerRadius = turningRadius_in - romiRadius;
    double leftTarget_in, rightTarget_in;

    if(isLeft) {
      leftTarget_in = arcLength_in * (innerRadius / turningRadius_in);
      rightTarget_in = arcLength_in * (outerRadius / turningRadius_in); 
    } else{
      leftTarget_in = arcLength_in * (outerRadius / turningRadius_in);
      rightTarget_in = arcLength_in * (innerRadius / turningRadius_in);
    }
    setMembers(arcLength_in, leftTarget_in, rightTarget_in, drivetrain);
  }

  private void setMembers(double middleTarget_in, double leftTarget_in, double rightTarget_in, RomiDrivetrain drivetrain) {
    m_middleTarget_in = middleTarget_in;
    m_leftTarget_in = leftTarget_in;
    m_rightTarget_in = rightTarget_in;
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
    m_leftPower = 0;
    m_rightPower = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the travelled distance for the middle of the robot.
    double arcTravelled_in = getArcTravelledInch();
    SmartDashboard.putNumber("Distance to go", m_middleTarget_in - arcTravelled_in);

    //Calculate the error and error percent for the target.
    double leftError_in = m_leftTarget_in - m_drivetrain.getLeftDistanceInch();
    double rightError_in = m_rightTarget_in - m_drivetrain.getRightDistanceInch();
    double leftErrorScaled = leftError_in / m_leftTarget_in;
    double rightErrorScaled = rightError_in / m_rightTarget_in;

    //Determine how far the left or right is off from the given arc.
    double turnError = (leftErrorScaled - rightErrorScaled) * (m_middleTarget_in - arcTravelled_in);

    //Outputs are determined by distance from the target and turn error (multiplied by gains).
    double leftOutput = (leftError_in * kForwardGain) + (turnError * kTurnGain);
    double rightOutput = (rightError_in * kForwardGain) - (turnError * kTurnGain);

    //Gets the absolute value for the outputs.
    double absLeftOutput = Math.abs(leftOutput);
    double absRightOutput = Math.abs(rightOutput);

    //Find the larger magnitude of the outputs.
    double smallerOutput = (absLeftOutput < absRightOutput)? absRightOutput : absLeftOutput;
    double biggerOutput = (absLeftOutput > absRightOutput)? absLeftOutput : absRightOutput;

    if(biggerOutput != 0) {
      if(absRightOutput < kMinPower || absLeftOutput < kMinPower) { 
        //If the output is too low to move, proportionally adjust the speeds.
        leftOutput *= kMinPower / smallerOutput;
        rightOutput *= kMinPower / smallerOutput;
      } else if(absRightOutput > kMaxPower || absLeftOutput > kMaxPower) {
        //If the output is moving too fast, proportionally adjust the speeds.
        leftOutput *= kMaxPower / biggerOutput;
        rightOutput *= kMaxPower / biggerOutput;
      }
    }

    //Get the larger target distance.
    double absLeftTarget_in = Math.abs(m_leftTarget_in);
    double absRightTarget_in = Math.abs(m_rightTarget_in);
    double biggerTarget_in = (absLeftTarget_in > absRightTarget_in)? absLeftTarget_in : absRightTarget_in;

    //The max acceleration is based proportionally based on which side needs to travel faster.
    double leftMaxAcceleration = kMaxPowerChange * (absLeftTarget_in / biggerTarget_in);
    double rightMaxAcceleration = kMaxPowerChange * (absRightTarget_in / biggerTarget_in);

    //Set the speeds but throttle velocity change over time.
    m_leftPower = throttleAcceleration(m_leftPower, leftOutput, leftMaxAcceleration);
    m_rightPower = throttleAcceleration(m_rightPower, rightOutput, rightMaxAcceleration);
    m_drivetrain.TankDrive(m_leftPower, m_rightPower);
  }

  private double throttleAcceleration(double currentPower, double proposedPower, double maxAcceleration) {
    boolean proposedPowerIsPositive = proposedPower > 0;

    //Figure out the power change in respect to 0.
    double powerChange = (proposedPowerIsPositive)? proposedPower - currentPower : Math.abs(currentPower - proposedPower);

    //Handle cases with 0.
    if(currentPower == 0) {
      powerChange = Math.abs(proposedPower);
    } else if(proposedPower == 0) {
      powerChange = -Math.abs(proposedPower);
    }

    //If power change is greater than the max acceleration.
    if (powerChange > maxAcceleration) {
      if ((proposedPower > 0 && currentPower < 0) || (proposedPower < 0 && currentPower > 0)) {
        //If current and proposed as opposite directions, set the power to 0.
        proposedPower = 0;
      } else {
        double addAcceleration = (proposedPowerIsPositive)? maxAcceleration : -maxAcceleration;
        proposedPower = currentPower + addAcceleration;
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
    return getArcTravelledInch() >= m_middleTarget_in - kTolerance_in;
  }

  private double getArcTravelledInch() {
    double leftDistance_in = m_drivetrain.getLeftDistanceInch();
    double rightDistance_in = m_drivetrain.getRightDistanceInch();
    double middleDistance_in = (leftDistance_in + rightDistance_in) / 2;
    return middleDistance_in;
  }
}

