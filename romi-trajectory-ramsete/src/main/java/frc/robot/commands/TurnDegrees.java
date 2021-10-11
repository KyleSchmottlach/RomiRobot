// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;

public class TurnDegrees extends PIDCommand {
  private final Drivetrain m_drive;
  private final double m_degrees;
  private final double m_speed;
  private int counter = 0;
  private boolean endingCheck;

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegrees(double speed, double degrees, Drivetrain drive) {
    super(new PIDController(0.0085, 0, 0), 
    drive::getGyroAngleZ, 
    degrees, 
    output -> {
      double clampedOutput = MathUtil.clamp(output, -1, 1);
      if(Math.abs(clampedOutput) < 0.325 && Math.abs(clampedOutput) > 0) {
        double tempClampedOutput = clampedOutput;
        clampedOutput = Math.copySign(0.325, tempClampedOutput);
      }
      System.out.println(clampedOutput);
      drive.arcadeDrive(0, clampedOutput);
    }, 
    drive);

    m_degrees = degrees;
    m_speed = speed;
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    
    m_drive.arcadeDrive(0, 0);
    m_drive.resetGyro();
    m_drive.resetOdometry(new Pose2d()); 
  }

  @Override
  public void execute() {
    super.execute();
    if(counter <= 15) {
      counter++;
    } else {
      counter = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* Need to convert distance travelled to degrees. The Standard
       Romi Chassis found here, https://www.pololu.com/category/203/romi-chassis-kits,
       has a wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
       or 5.551 inches. We then take into consideration the width of the tires.
    */
    //double inchPerDegree = Math.PI * 5.551 / 360;
    // Compare distance travelled from start to distance based on degree turn

    if(counter == 4) {
      if(m_degrees != 0) {
        return m_drive.getGyroAngleZ() >= m_degrees - 7 && m_drive.getGyroAngleZ() <= m_degrees + 7;
      } else {
        return true;
      }
    } else {
      return false;
    }
  }
}
