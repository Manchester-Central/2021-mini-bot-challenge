// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArcDrive;
import frc.robot.commands.ArcDriveVelocity;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DistanceAutoDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.PidDrive;
import frc.robot.commands.PidTurn;
import frc.robot.data.Direction;
import frc.robot.gamepads.Gamepad;
import frc.robot.subsystems.RomiDrivetrain;

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
  private DigitalOutput redLed = new DigitalOutput(2);
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  public Gamepad Driver = new Gamepad(0, "Driver");
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_romiDrivetrain);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_romiDrivetrain.setDefaultCommand(new ArcadeDrive(Driver, m_romiDrivetrain));
    Driver.getButtonStart().whenPressed(() -> m_romiDrivetrain.resetEncoders());
    Driver.getButtonRB().whileActiveOnce(new PidDrive(12, 12, m_romiDrivetrain));
    Driver.getButtonLB().whileActiveOnce(new PidDrive(-12, -12, m_romiDrivetrain));
    Driver.getButtonRT().whileActiveOnce(new PidTurn(90, m_romiDrivetrain));
    Driver.getButtonLT().whileActiveOnce(new PidTurn(-90, m_romiDrivetrain));
    Driver.getButtonY().whileActiveOnce(new SequentialCommandGroup(
      new PidDrive(17, 17, m_romiDrivetrain),
      new PidTurn(-90, m_romiDrivetrain),
      new PidDrive(17, 17, m_romiDrivetrain),
      new PidTurn(-90, m_romiDrivetrain),
      new PidDrive(13.5, 13.5, m_romiDrivetrain),
      new PidTurn(-90, m_romiDrivetrain),
      new PidDrive(9, 9, m_romiDrivetrain),
      new PidTurn(90, m_romiDrivetrain),
      new PidDrive(13.5, 13.5, m_romiDrivetrain),
      new PidTurn(90, m_romiDrivetrain),
      new PidDrive(17, 17, m_romiDrivetrain),
      new PidTurn(90, m_romiDrivetrain),
      new PidDrive(17, 17, m_romiDrivetrain)
    ));
    Driver.getButtonSelect().whileActiveOnce(new SequentialCommandGroup(
      new DistanceAutoDrive(4.3, m_romiDrivetrain),
      new ArcDrive(13.303, 20.897, Direction.Left, m_romiDrivetrain),
      new ArcDrive(5.5, 8.8, Direction.Left, m_romiDrivetrain),
      new DistanceAutoDrive(13, m_romiDrivetrain),
      new ArcDrive(5.5, 8.8, Direction.Right, m_romiDrivetrain),
      new ArcDrive(13.303, 19.4, Direction.Right, m_romiDrivetrain),
      new DistanceAutoDrive(6.5, m_romiDrivetrain)
    ));
    Driver.getButtonX().whileActiveOnce(new SequentialCommandGroup(
      new DistanceAutoDrive(4.3, m_romiDrivetrain),
      new ArcDrive(13.303, 20.897, Direction.Left, m_romiDrivetrain),
      new ArcDrive(4.3, 8.905, Direction.Left, m_romiDrivetrain),
      new DistanceAutoDrive(15.809, m_romiDrivetrain),
      new ArcDrive(4.3, 8.905, Direction.Right, m_romiDrivetrain),
      new ArcDrive(13.303, 20.897, Direction.Right, m_romiDrivetrain),
      new DistanceAutoDrive(4.3, m_romiDrivetrain)
    ));
    Driver.getButtonB().whileActiveOnce(new SequentialCommandGroup(
      ArcDriveVelocity.GetResetCommand(),
      new ArcDriveVelocity(4.3, m_romiDrivetrain),
      new ArcDriveVelocity(13.303, 20.897, Direction.Left, m_romiDrivetrain),
      new ArcDriveVelocity(4.3, 8.9, Direction.Left, m_romiDrivetrain),
      new ArcDriveVelocity(15.809, m_romiDrivetrain),
      new ArcDriveVelocity(4.3, 8.9, Direction.Right, m_romiDrivetrain),
      new ArcDriveVelocity(13.303, 20.897, Direction.Right, m_romiDrivetrain),
      new ArcDriveVelocity(4.3, m_romiDrivetrain)
    ));
    Driver.getButtonA().whileActiveOnce(new SequentialCommandGroup(
      ArcDriveVelocity.GetResetCommand(),
      new ArcDriveVelocity(4.3, m_romiDrivetrain),
      new ArcDriveVelocity(13.303, 20.897, Direction.Left, m_romiDrivetrain),
      new ArcDriveVelocity(5.5, 8.8, Direction.Left, m_romiDrivetrain),
      new ArcDriveVelocity(13, m_romiDrivetrain),
      new ArcDriveVelocity(5.5, 8.8, Direction.Right, m_romiDrivetrain),
      new ArcDriveVelocity(13.303, 19.4, Direction.Right, m_romiDrivetrain),
      new ArcDriveVelocity(6.5, m_romiDrivetrain)
    ));
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
