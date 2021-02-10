// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TimedAutoDrive;
import frc.robot.commands.ToggleLED;
import frc.robot.subsystems.PushButton;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  PushButton m_buttonA = new PushButton(0);
  public Joystick Driver = new Joystick(0);
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
    m_romiDrivetrain.setDefaultCommand(new TankDrive(Driver, m_romiDrivetrain));
    JoystickButton a = new JoystickButton(Driver, 1);
    a.whenPressed(new ToggleLED(greenLed));
    JoystickButton x = new JoystickButton(Driver, 3);
    x.toggleWhenPressed(new ArcadeDrive(Driver, m_romiDrivetrain));
    JoystickButton b = new JoystickButton(Driver, 2);
    b.whenPressed(new TimedAutoDrive(4, m_romiDrivetrain, 0.35, 0.351));
    JoystickButton y = new JoystickButton(Driver, 4);
    TimedAutoDrive driveforward = new TimedAutoDrive(3, m_romiDrivetrain, 0.35, 0.35);
    TimedAutoDrive drivebackward = new TimedAutoDrive(3, m_romiDrivetrain, -0.35, -0.35);
    y.and(b).whenActive(new SequentialCommandGroup(driveforward, drivebackward));
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
