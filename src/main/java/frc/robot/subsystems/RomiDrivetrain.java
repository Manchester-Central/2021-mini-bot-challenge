// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

private PIDController LeftPidController = new PIDController(0, 0, 0);
private PIDController rightPidController = new PIDController(0, 0, 0);
  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    arcadeDrive(xaxisSpeed, zaxisRotate, false);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate, boolean smoothing) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate, smoothing);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Inches", getLeftDistanceInch());
    SmartDashboard.putNumber("Right Inches", getRightDistanceInch());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void TankDrive(double leftPower, double rightPower) {
   TankDrive(leftPower, rightPower, false);
  }

  public void TankDrive(double leftPower, double rightPower, boolean smoothing) {
    m_diffDrive.tankDrive(leftPower, rightPower, smoothing);
  }

public void PidDrive(){
double PowerLeft = LeftPidController.calculate(getLeftDistanceInch());
double PowerRight = rightPidController.calculate(getRightDistanceInch());
TankDrive(PowerLeft, PowerRight);
}
public void SetPidTarget(double TargetLeft_in, double TargetRight_in){
LeftPidController.setSetpoint(TargetLeft_in);
rightPidController.setSetpoint(TargetRight_in);
}
public boolean TargetReached(){
return true; //TO DO
}
}
