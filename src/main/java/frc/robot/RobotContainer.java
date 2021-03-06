// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.ArcDrive;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoCorrectedDrive;
import frc.robot.commands.DistanceAutoDrive;
import frc.robot.commands.PidDrive;
import frc.robot.commands.PidGyro;
import frc.robot.commands.PidTurn;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TimedAutoDrive;
import frc.robot.commands.ToggleLED;
import frc.robot.gamepads.Gamepad;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DigitalOutput greenLed = new DigitalOutput(1);
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  public Gamepad Driver = new Gamepad(0, "Driver");
  private final Command m_autoCommand = new SequentialCommandGroup(new AutoCorrectedDrive(6.803, m_romiDrivetrain),
      new AutoCorrectedDrive(13.303, 20.897, true, m_romiDrivetrain),
      new AutoCorrectedDrive(4.303, 8.205, true, m_romiDrivetrain), new AutoCorrectedDrive(17.709, m_romiDrivetrain),
      new AutoCorrectedDrive(4.303, 8.905, false, m_romiDrivetrain),
      new AutoCorrectedDrive(13.850, 20.987, false, m_romiDrivetrain), new AutoCorrectedDrive(2.5, m_romiDrivetrain));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

  }

  private Command getAutoDriveCommand() {
    String trajectoryJSON = "paths/output/AutoPath.wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory" + trajectoryJSON, ex.getStackTrace());
      return new RunCommand(() -> {});
    }
  
    Command command = new RamseteCommand(
      trajectory,
      m_romiDrivetrain::getPose2d,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerInch, Constants.kaVoltSecondsSquaredPerInch),
      new DifferentialDriveKinematics(RomiDrivetrain.kTrackWidthInch),
      m_romiDrivetrain::getWheelSpeeds,
      new PIDController(Constants.kRamseteP, Constants.kRamseteI, Constants.kRamseteD),
      new PIDController(Constants.kRamseteP, Constants.kRamseteI, Constants.kRamseteD),
      m_romiDrivetrain::tankDriveVolts,
      m_romiDrivetrain);

    final Pose2d initialPose = trajectory.getInitialPose();
    m_romiDrivetrain.resetOdometry(trajectory.getInitialPose());
    return new InstantCommand(() -> m_romiDrivetrain.resetOdometry(initialPose), m_romiDrivetrain)
    .andThen(command)
    .andThen(new InstantCommand(() -> m_romiDrivetrain.tankDriveVolts(0.0, 0.0), m_romiDrivetrain));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_romiDrivetrain.setDefaultCommand(new TankDrive(Driver, m_romiDrivetrain));
    Driver.getButtonA().whenPressed(getAutoDriveCommand());
    Driver.getButtonX().toggleWhenPressed(new ArcadeDrive(Driver, m_romiDrivetrain));
    Button b = Driver.getButtonB();
    b.whileActiveOnce(new SequentialCommandGroup(new DistanceAutoDrive(4.303, m_romiDrivetrain),
        new ArcDrive(13.303, 20.897, true, m_romiDrivetrain), new ArcDrive(5.5, 8, true, m_romiDrivetrain),
        new DistanceAutoDrive(10.85, m_romiDrivetrain), new ArcDrive(5.5, 10.4, false, m_romiDrivetrain),
        new ArcDrive(13.303, 21, false, m_romiDrivetrain), new DistanceAutoDrive(4.303, m_romiDrivetrain)));
    Button y = Driver.getButtonY();
    TimedAutoDrive driveforward = new TimedAutoDrive(3, m_romiDrivetrain, 0.35, 0.35);
    TimedAutoDrive drivebackward = new TimedAutoDrive(3, m_romiDrivetrain, -0.35, -0.35);
    y.and(b).whenActive(new SequentialCommandGroup(driveforward, drivebackward));
    Driver.getButtonStart().whenPressed(() -> m_romiDrivetrain.resetEncoders());
    Driver.getButtonSelect().whenPressed(new DistanceAutoDrive(12, m_romiDrivetrain));
    Driver.getButtonRB().whileActiveOnce(m_autoCommand);
    Driver.getButtonLB().whileActiveOnce(new PidDrive(-12, -12, m_romiDrivetrain));
    Driver.getButtonRT().whileActiveOnce(new PidGyro(90, m_romiDrivetrain));
    Driver.getButtonLT().whileActiveOnce(new PidTurn(-90, m_romiDrivetrain));
    y.whileActiveOnce(new SequentialCommandGroup(new PidDrive(17, 17, m_romiDrivetrain),
        new PidTurn(-90, m_romiDrivetrain), new PidDrive(17, 17, m_romiDrivetrain), new PidTurn(-90, m_romiDrivetrain),
        new PidDrive(13.5, 13.5, m_romiDrivetrain), new PidTurn(-90, m_romiDrivetrain),
        new PidDrive(9, 9, m_romiDrivetrain), new PidTurn(90, m_romiDrivetrain),
        new PidDrive(13.5, 13.5, m_romiDrivetrain), new PidTurn(90, m_romiDrivetrain),
        new PidDrive(17, 17, m_romiDrivetrain), new PidTurn(90, m_romiDrivetrain),
        new PidDrive(17, 17, m_romiDrivetrain)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
// ⡿⠉⠄⠄⠄⠄⠈⠙⠿⠟⠛⠉⠉⠉⠄⠄⠄⠈⠉⠉⠉⠛⠛⠻⢿⣿⣿⣿⣿⣿
// ⠁⠄⠄⠄⢀⡴⣋⣵⣮⠇⡀⠄⠄⠄⠄⠄⠄⢀⠄⠄⠄⡀⠄⠄⠄⠈⠛⠿⠋⠉
// ⠄⠄⠄⢠⣯⣾⣿⡿⣳⡟⣰⣿⣠⣂⡀⢀⠄⢸⡄⠄⢀⣈⢆⣱⣤⡀⢄⠄⠄⠄
// ⠄⠄⠄⣼⣿⣿⡟⣹⡿⣸⣿⢳⣿⣿⣿⣿⣴⣾⢻⣆⣿⣿⣯⢿⣿⣿⣷⣧⣀⣤
// ⠄⠄⣼⡟⣿⠏⢀⣿⣇⣿⣏⣿⣿⣿⣿⣿⣿⣿⢸⡇⣿⣿⣿⣟⣿⣿⣿⣿⣏⠋
// ⡆⣸⡟⣼⣯⠏⣾⣿⢸⣿⢸⣿⣿⣿⣿⣿⣿⡟⠸⠁⢹⡿⣿⣿⢻⣿⣿⣿⣿⠄
// ⡇⡟⣸⢟⣫⡅⣶⢆⡶⡆⣿⣿⣿⣿⣿⢿⣛⠃⠰⠆⠈⠁⠈⠙⠈⠻⣿⢹⡏⠄
// ⣧⣱⡷⣱⠿⠟⠛⠼⣇⠇⣿⣿⣿⣿⣿⣿⠃⣰⣿⣿⡆⠄⠄⠄⠄⠄⠉⠈⠄⠄
// ⡏⡟⢑⠃⡠⠂⠄⠄⠈⣾⢻⣿⣿⡿⡹⡳⠋⠉⠁⠉⠙⠄⢀⠄⠄⠄⠄⠄⠂⠄
// ⡇⠁⢈⢰⡇⠄⠄⡙⠂⣿⣿⣿⣿⣱⣿⡗⠄⠄⠄⢀⡀⠄⠈⢰⠄⠄⠄⠐⠄⠄
// ⠄⠄⠘⣿⣧⠴⣄⣡⢄⣿⣿⣿⣷⣿⣿⡇⢀⠄⠤⠈⠁⣠⣠⣸⢠⠄⠄⠄⠄⠄
// ⢀⠄⠄⣿⣿⣷⣬⣵⣿⣿⣿⣿⣿⣿⣿⣷⣟⢷⡶⢗⡰⣿⣿⠇⠘⠄⠄⠄⠄⠄
// ⣿⠄⠄⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣶⣾⣿⣿⡟⢀⠃⠄⢸⡄⠁⣸
// ⣿⠄⠄⠘⢿⣿⣿⣿⣿⣿⣿⢛⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⢄⡆⠄⢀⣪⡆⠄⣿
// ⡟⠄⠄⠄⠄⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⢿⣟⣻⣩⣾⣃⣴⣿⣿⡇⠸⢾