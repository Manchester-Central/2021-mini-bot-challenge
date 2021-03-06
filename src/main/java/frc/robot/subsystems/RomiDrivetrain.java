// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;

public class RomiDrivetrain extends SubsystemBase {

  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeter = 0.07;
  private static final double kTurningCircumferenceInch = Constants.kTrackWidthInch * Math.PI;

  // Drive straight PID values
  private static final double kStraightP = 0.15;
  private static final double kStraightI = 0.6;
  private static final double kStraightD = 0;
  // private static final double kStraightP = 0.3;
  // private static final double kStraightI = 0;
  // private static final double kStraightD = 0;
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
  private final DifferentialDriveOdometry m_odometry;

  private final Field2d m_field2d = new Field2d();

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    resetEncoders();
    leftPidController.setTolerance(0.1);
    rightPidController.setTolerance(0.1);
    SmartDashboard.putData("leftPID", leftPidController);
    SmartDashboard.putData("rightPID", rightPidController);
    m_romiGyro.reset();
    m_odometry = new DifferentialDriveOdometry(m_romiGyro.getRotation2d());
    SmartDashboard.putData("Field2d", m_field2d);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    arcadeDrive(xaxisSpeed, zaxisRotate, false);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate, boolean smoothing) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate, smoothing);
  }

  public double getAngle() {
    return m_romiGyro.getAngleZ();
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance() * Constants.kInchPerMeter;
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance() * Constants.kInchPerMeter;
  }
  public double getLeftDistanceMeter() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeter() {
    return m_rightEncoder.getDistance();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Meters per second", m_leftEncoder.getRate()); 
    SmartDashboard.putNumber("Right Meters per second", m_rightEncoder.getRate());

    SmartDashboard.putNumber("Left Meters", getLeftDistanceMeter());
    SmartDashboard.putNumber("Right Meters", getRightDistanceMeter());

    SmartDashboard.putNumber("Left Inches", getLeftDistanceInch());
    SmartDashboard.putNumber("Right Inches", getRightDistanceInch());
    SmartDashboard.putNumber("Gyro X", m_romiGyro.getAngleX());
    SmartDashboard.putNumber("Gyro Y", m_romiGyro.getAngleY());
    SmartDashboard.putNumber("Gyro Z", m_romiGyro.getAngleZ());

    m_odometry.update(m_romiGyro.getRotation2d(), getLeftDistanceMeter(), getRightDistanceMeter());
    // m_romiGyro.

    Pose2d realPosition = m_odometry.getPoseMeters();
    double deg = realPosition.getRotation().unaryMinus().getDegrees();
    Rotation2d angle = Rotation2d.fromDegrees(deg + 45);

    Translation2d translation2d = new Translation2d(Math.sqrt((1.5*1.5) + (1.5*1.5)), angle );
    Rotation2d rotation2d = new Rotation2d(0);
    Transform2d transform2d = new Transform2d(translation2d, rotation2d);

    Pose2d pose2d = m_odometry.getPoseMeters().transformBy(transform2d);
    m_field2d.setRobotPose(pose2d);
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

  public Pose2d getPose2d() {
    // returns values in inches
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(-rightVolts);
    m_diffDrive.feed();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_romiGyro.getRotation2d());
  }
}
