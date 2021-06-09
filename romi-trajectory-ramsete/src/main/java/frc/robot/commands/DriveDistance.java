// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;

public class DriveDistance extends PIDCommand {
  private final Drivetrain m_drive;
  private final double m_distance;
  private final double m_speed;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double speed, double meters, Drivetrain drive) {
    super(
      new PIDController(0.12, 0, 0), 
      drive::getGyroAngleZ, 
      0,
      output -> {
        double clampedOutput = MathUtil.clamp(output, -1, 1);
        if(Math.abs(clampedOutput) < 0.125 && Math.abs(clampedOutput) > 0) {
          double tempClampedOutput = clampedOutput;
          clampedOutput = Math.copySign(0.125, tempClampedOutput);
        }
        drive.arcadeDrive(speed, clampedOutput);
      },
      drive
    );

    m_distance = meters;
    m_speed = speed;
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    m_drive.resetGyro();
    m_drive.resetOdometry(new Pose2d());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
    System.out.println("Command Finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(m_drive.getPose().getX()) >= m_distance;
  }
}
