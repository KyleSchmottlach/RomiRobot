// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;
  private final Supplier<Double> leftTankDrive;
  private final Supplier<Double> rightTankDrive;
  private final Supplier<Boolean> switchSides;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * @param zaxisRotateSuppplier Lambda supplier of rotational speed
   */
  public ArcadeDrive(
      Drivetrain drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSuppplier,
      Supplier<Double> leftTankDrive,
      Supplier<Double> rightTankDrive,
      Supplier<Boolean> switchSides) {
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSuppplier;
    this.leftTankDrive = leftTankDrive;
    this.rightTankDrive = rightTankDrive;
    this.switchSides = switchSides;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(switchSides.get()) {
      Robot.getRobotContainer().flipTeleOpDriveSide();
    } 
    if (leftTankDrive.get() > 0.0 || rightTankDrive.get() > 0.0) {
      m_drivetrain.tankDrive(Robot.getRobotContainer().getTeleOpDriveSide() * leftTankDrive.get(), 
      Robot.getRobotContainer().getTeleOpDriveSide() * -rightTankDrive.get());
    } else {
      m_drivetrain.arcadeDrive(Robot.getRobotContainer().getTeleOpDriveSide() * m_xaxisSpeedSupplier.get(), 0.7 * m_zaxisRotateSupplier.get());
    }
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
