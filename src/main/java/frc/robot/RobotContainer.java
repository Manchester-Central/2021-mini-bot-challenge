// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DistanceAutoDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.PidDrive;
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
    m_romiDrivetrain.setDefaultCommand(new TankDrive(Driver, m_romiDrivetrain));
    Driver.getButtonA().whenPressed(new ToggleLED(greenLed));
    Driver.getButtonX().toggleWhenPressed(new ArcadeDrive(Driver, m_romiDrivetrain));
    Button b = Driver.getButtonB();
    b.whenPressed(new TimedAutoDrive(4, m_romiDrivetrain, 0.35, 0.351));
    Button y = Driver.getButtonY();
    TimedAutoDrive driveforward = new TimedAutoDrive(3, m_romiDrivetrain, 0.35, 0.35);
    TimedAutoDrive drivebackward = new TimedAutoDrive(3, m_romiDrivetrain, -0.35, -0.35);
    y.and(b).whenActive(new SequentialCommandGroup(driveforward, drivebackward));
    Driver.getButtonStart().whenPressed(() -> m_romiDrivetrain.resetEncoders());
    Driver.getButtonSelect().whenPressed(new DistanceAutoDrive(12, m_romiDrivetrain));
    Driver.getButtonRB().whileActiveOnce(new PidDrive(12, 12, m_romiDrivetrain));
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
