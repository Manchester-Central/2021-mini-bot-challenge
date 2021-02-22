// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.RomiGyro;

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm
  private static final double kTrackWidthInch = 5.55;
  private static final double kTurningCircumferenceInch = kTrackWidthInch * Math.PI;

  // Drive straight PID values
  private static final double kStraightP = 0.15;
  private static final double kStraightI = 0.6;
  private static final double kStraightD = 0;
  private static final TrapezoidProfile.Constraints kStraightConstraints = new TrapezoidProfile.Constraints(8, 10);

  // Drive turn PID values
  private static final double kTurnP = 0.3;
  private static final double kTurnI = 0.12;
  private static final double kTurnD = 0;
  private static final TrapezoidProfile.Constraints kTurnConstraints = new TrapezoidProfile.Constraints(10, 10);

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
  private ProfiledPIDController leftPidController = new ProfiledPIDController(kStraightP, kStraightI, kStraightD,
      kStraightConstraints);
  private ProfiledPIDController rightPidController = new ProfiledPIDController(kStraightP, kStraightI, kStraightD,
      kStraightConstraints);
  // private PIDController leftPidController = new PIDController(0.2, 0, 0);
  // private PIDController rightPidController = new PIDController(0.2, 0, 0);

  private final RomiGyro m_romiGyro = new RomiGyro();

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
    leftPidController.setTolerance(0.1);
    rightPidController.setTolerance(0.1);
    SmartDashboard.putData("leftPID", leftPidController);
    SmartDashboard.putData("rightPID", rightPidController);
    m_romiGyro.reset();
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
    SmartDashboard.putNumber("Gyro X", m_romiGyro.getAngleX());
    SmartDashboard.putNumber("Gyro Y", m_romiGyro.getAngleY());
    SmartDashboard.putNumber("Gyro Z", m_romiGyro.getAngleZ());
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

  public void PidDrive() {
    double PowerLeft = leftPidController.calculate(getLeftDistanceInch());
    double PowerRight = rightPidController.calculate(getRightDistanceInch());
    TankDrive(PowerLeft, PowerRight);
    SmartDashboard.putNumber("PowerLeft", PowerLeft);
    SmartDashboard.putNumber("PowerRight", PowerRight);
  }

  private void SetPidTarget(double TargetLeft_in, double TargetRight_in) {
    leftPidController.reset(getLeftDistanceInch());
    rightPidController.reset(getRightDistanceInch());

    leftPidController.setGoal(TargetLeft_in);
    rightPidController.setGoal(TargetRight_in);
    // leftPidController.setSetpoint(TargetLeft_in);
    // rightPidController.setSetpoint(TargetRight_in);
  }

  public void SetPidTargetDistance(double TargetLeft_in, double TargetRight_in) {
    leftPidController.setPID(kStraightP, kStraightI, kStraightD);
    rightPidController.setPID(kStraightP, kStraightI, kStraightD);
    leftPidController.setConstraints(kStraightConstraints);
    rightPidController.setConstraints(kStraightConstraints);
    SetPidTarget(getLeftDistanceInch() + TargetLeft_in, getRightDistanceInch() + TargetRight_in);
  }

  public void SetPidTargetAngle(double TargetAngle) {
    leftPidController.setPID(kTurnP, kTurnI, kTurnD);
    rightPidController.setPID(kTurnP, kTurnI, kTurnD);
    leftPidController.setConstraints(kTurnConstraints);
    rightPidController.setConstraints(kTurnConstraints);
    double deltaDistance = (TargetAngle / 360) * kTurningCircumferenceInch;
    SetPidTarget(getLeftDistanceInch() + deltaDistance, getRightDistanceInch() - deltaDistance);

  }

  public boolean TargetReached() {
    return leftPidController.atGoal() && rightPidController.atGoal();
    // return leftPidController.atSetpoint() && rightPidController.atSetpoint();

  }
}
