// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {

  private final Intake m_intake;
  private final double m_direction;

  /** Creates a new SetIntakePower. */
  public RunIntake(Intake intake, boolean input) {

    m_intake = intake;
	if (input) {
		m_direction = 1.0;
	}
	else {
		m_direction = -1.0;
	}
    SmartDashboard.setDefaultNumber("intakePower", 1.0);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double intakePower = SmartDashboard.getNumber("intakePower", 1.0);
    m_intake.setPower(m_direction * MathUtil.clamp(intakePower, -1, 1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
