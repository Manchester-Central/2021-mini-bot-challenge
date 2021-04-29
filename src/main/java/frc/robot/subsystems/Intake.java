// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final PWM m_PwmFront;
  private final PWM m_PwmBack;

  /** Creates a new Intake. */
  public Intake(int channelFront, int channelBack) {
    m_PwmFront = new PWM(channelFront);
    m_PwmBack = new PWM(channelBack);
  }

  public void setPower(double power) {
    m_PwmFront.setSpeed(power);
    m_PwmBack.setSpeed(power);
    SmartDashboard.putNumber("ActualIntakePower", power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
