// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

public class Collector extends SubsystemBase {
  
  private final DigitalOutput m_pwmA1;
  private final DigitalOutput m_pwmA2;

  public Collector() {
    m_pwmA1 = new DigitalOutput(10);
    m_pwmA2 = new DigitalOutput(11);

    reset();
  }

  public void reset() {
    m_pwmA1.set(false);
    m_pwmA2.set(false);
  }

  public void setSpeed(double speed) {
    if(speed >= 0.1) {
      m_pwmA1.set(true);
      m_pwmA2.set(false);
    } else if (speed <= -0.1) {
      m_pwmA1.set(false);
      m_pwmA2.set(true);
    } else {
      m_pwmA1.set(false);
      m_pwmA2.set(false);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
