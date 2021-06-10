// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class IntakeCommand extends CommandBase {
  private Collector collector;
  private double speed;
  private boolean requiresTime = false;
  private double time;
  private double startTime;
  private boolean instant = false;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(double speed, Collector collector) {
    this.collector = collector;
    this.speed = speed;
    addRequirements(collector);
  }

  public IntakeCommand(double speed, boolean instant, Collector collector) {
    this(speed, collector);
    this.instant = instant;
  }

  public IntakeCommand(double speed, double time, Collector collector){ 
    this(speed, collector);
    requiresTime = true;
    this.time = time*1000;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Intaking with speed " + speed);
    if(requiresTime) {
      startTime = System.currentTimeMillis();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    collector.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ending");
    if(!instant) {
      collector.setSpeed(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(instant) {
      return true;
    }
    if(requiresTime) {
      return System.currentTimeMillis() - startTime >= time;
    }
    return false;
  }
}
