// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;

import java.util.function.Supplier;

public class ArcadeDrive extends PIDCommand {
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
   * @param zaxisRotateSupplier Lambda supplier of rotational speed
   */
  public ArcadeDrive(
      Drivetrain drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSupplier,
      Supplier<Double> leftTankDrive,
      Supplier<Double> rightTankDrive,
      Supplier<Boolean> switchSides) {

    super(
      new PIDController(0.025, 0, 0), 
      drivetrain::getGyroAngleZ, 
      0, 
      output -> {
        double clampedOutput = MathUtil.clamp(output, -1, 1);

        /*if(switchSides.get()) {
          Robot.getRobotContainer().flipTeleOpDriveSide();
        } */

        if(Math.abs(clampedOutput) < 0.2){
          double tempClampedOutput = clampedOutput;
          clampedOutput = Math.copySign(0.2, tempClampedOutput);
        }

        //System.out.println("Clamped Output: " + clampedOutput);
        //drivetrain.arcadeDrive(Robot.getRobotContainer().getTeleOpDriveSide() * xaxisSpeedSupplier.get(), clampedOutput);

        /*if(zaxisRotateSupplier.get() >= 0.05 || zaxisRotateSupplier.get() <= -0.05) {
          System.out.println("Turning");
          drivetrain.arcadeDrive(Robot.getRobotContainer().getTeleOpDriveSide() * xaxisSpeedSupplier.get(), 0.9 * zaxisRotateSupplier.get());
        } else {
          System.out.println("Clamped Output: " + clampedOutput);
          drivetrain.arcadeDrive(Robot.getRobotContainer().getTeleOpDriveSide() * xaxisSpeedSupplier.get(), clampedOutput);
        }*/
        drivetrain.arcadeDrive(Robot.getRobotContainer().getTeleOpDriveSide() * xaxisSpeedSupplier.get(), 0.9 * zaxisRotateSupplier.get());
      }, 
      drivetrain);
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSupplier;
    this.leftTankDrive = leftTankDrive;
    this.rightTankDrive = rightTankDrive;
    this.switchSides = switchSides;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    m_drivetrain.resetOdometry(new Pose2d());
    m_drivetrain.resetGyro();
    m_drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    /*if (m_zaxisRotateSupplier.get() <= -0.05 || m_zaxisRotateSupplier.get() >= 0.05) {
      getController().setSetpoint(m_drivetrain.getGyroAngleZ());
      System.out.println("Resetting Setpoint: " + getController().getSetpoint());
    }*/
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
