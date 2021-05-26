// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import javax.sound.midi.Patch;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.ArcDrive;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DistanceAutoDrive;
import frc.robot.commands.PathDrive;
import frc.robot.commands.PidDrive;
import frc.robot.commands.PidGyro;
import frc.robot.commands.PidTurn;
import frc.robot.commands.RunIntake;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TimedAutoDrive;
import frc.robot.commands.ToggleLED;
import frc.robot.gamepads.Gamepad;
import frc.robot.subsystems.Intake;
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
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  private final Intake m_intake = new Intake(3, 2);
  public Gamepad Driver = new Gamepad(0, "Driver");

  private final Command m_autoFranticFetchPath = new SequentialCommandGroup(
    new PathDrive("AutoNavFranticFetch1", m_romiDrivetrain),
    new ToggleLED(greenLed),
    new PathDrive("AutoNavFranticFetch2", m_romiDrivetrain),
    new ToggleLED(greenLed),
    new PathDrive("AutoNavFranticFetch3", m_romiDrivetrain),
    new ToggleLED(greenLed),
    new PathDrive("AutoNavFranticFetch4", m_romiDrivetrain),
    new ToggleLED(greenLed)
  ); 

  private final Command m_AllianceAnticsGuaranteedRpCommand = new PathDrive("AllianceAnticsGuaranteedRpCommand", m_romiDrivetrain);

  private final Command m_autoAndIntakeCommand = new ParallelCommandGroup(
   m_autoFranticFetchPath, new RunIntake(m_intake, true)
  );


  private final Command m_autoParkOnly = new PathDrive("AllianceAnticsParkOnly", m_romiDrivetrain);

  private final Command m_AllianceAntics5BallsCommand = new PathDrive("AllianceAntics5BallsCommand", m_romiDrivetrain);
  private final Command m_AllianceAnticsAllBallsCommand = new PathDrive("AllianceAnticsAllBallsCommand", m_romiDrivetrain);
  
  private final Command m_autoStraightBluePath = new SequentialCommandGroup(
    new PathDrive("AllianceAnticsStraightBlue1", m_romiDrivetrain),
    new PathDrive("AllianceAnticsBluePark", m_romiDrivetrain)
  );
  private final Command m_autoStraightBlueCommand = new ParallelCommandGroup(
    m_autoStraightBluePath, new RunIntake(m_intake, true)
   );

  private final SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();


 // PathDrive("TurnTest", m_romiDrivetrain)
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_autoSelector.setDefaultOption("Park Only", m_autoParkOnly);
    m_autoSelector.addOption("Frantic Fetch", m_autoAndIntakeCommand);
    m_autoSelector.addOption("Straight Blue", m_autoStraightBlueCommand);
    m_autoSelector.addOption("AllianceAnticsGuaranteedRP", m_AllianceAnticsGuaranteedRpCommand);
    m_autoSelector.addOption("AllianceAntics5Balls", m_AllianceAntics5BallsCommand);
    m_autoSelector.addOption("AllianceAnticsAllBalls", m_AllianceAnticsAllBallsCommand);
    m_autoSelector.addOption("None", new RunCommand(() -> m_romiDrivetrain.TankDrive(0,0), m_romiDrivetrain));

    SmartDashboard.putData(m_autoSelector);
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_romiDrivetrain.setDefaultCommand(new ArcadeDrive(Driver, m_romiDrivetrain));
    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0), m_intake));
    Driver.getButtonA().whenPressed(m_autoAndIntakeCommand);
    Driver.getButtonX().toggleWhenPressed(new TankDrive(Driver, m_romiDrivetrain));
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
    Driver.getButtonRB().whileActiveOnce(new RunIntake(m_intake, false));
    Driver.getButtonLB().whileActiveOnce(new RunIntake(m_intake, true));
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
    return m_autoSelector.getSelected();
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