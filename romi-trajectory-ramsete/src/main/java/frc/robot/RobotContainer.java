// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TurnDegrees;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain;
  private final Collector collector;
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  // Assumes a gamepad plugged into channnel 0
  private final XboxController m_controller;

  //Xbox Controller Buttons
  private final JoystickButton drA;
  private final JoystickButton drB;
  private final JoystickButton drBumpLeft;
  private final JoystickButton drBumpRight;

  private SequentialCommandGroup franticFetchCommandGroup;
  private SequentialCommandGroup allianceAnticsCommandGroup;
  private SequentialCommandGroup straightCommandGroup;

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Supplier<Command>> m_chooser = new SendableChooser<>();

  private final double dxy = Units.inchesToMeters(7.5);

  private Trajectory mondrianMadnessTrajectory;
  private Trajectory barrelRaceTrajectory;
  private Trajectory slalomRaceTrajectory;
  private Trajectory bounceTrajectory1;
  private Trajectory bounceTrajectory2;
  private Trajectory bounceTrajectory3;
  private Trajectory bounceTrajectory4;
  private Trajectory forwardTrajectory;

  private int teleopDriveSide = 1;

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrain = new Drivetrain();
    collector = new Collector();

    m_controller = new XboxController(0);

    drA = new JoystickButton(m_controller, XboxController.Button.kA.value);
    drB = new JoystickButton(m_controller, XboxController.Button.kB.value);
    drBumpLeft = new JoystickButton(m_controller, XboxController.Button.kBumperLeft.value);
    drBumpRight = new JoystickButton(m_controller, XboxController.Button.kBumperRight.value);

    configureButtonBindings();
    generateTrajectories();
  
    franticFetchCommandGroup = new SequentialCommandGroup(
        new InstantCommand(() -> m_drivetrain.resetOdometry(bounceTrajectory1.getInitialPose())),
        ramseteCommandForTrajectory(bounceTrajectory1),
        new InstantCommand(() -> m_drivetrain.resetOdometry(bounceTrajectory2.getInitialPose())),
        ramseteCommandForTrajectory(bounceTrajectory2),
        new InstantCommand(() -> m_drivetrain.resetOdometry(bounceTrajectory3.getInitialPose())),
        ramseteCommandForTrajectory(bounceTrajectory3),
        new InstantCommand(() -> m_drivetrain.resetOdometry(bounceTrajectory4.getInitialPose())),
        ramseteCommandForTrajectory(bounceTrajectory4)
      );

    allianceAnticsCommandGroup = new SequentialCommandGroup(
      new IntakeCommand(1, true, collector),
      new DriveDistance(0.8, Units.inchesToMeters(12), m_drivetrain),
      new TurnDegrees(0.8, -90, m_drivetrain),
      new DriveDistance(0.8, Units.inchesToMeters(27), m_drivetrain),
      new TurnDegrees(0.8, 90, m_drivetrain),
      new DriveDistance(0.8, Units.inchesToMeters(27.5), m_drivetrain),
      new TurnDegrees(0.8, 55, m_drivetrain),
      new DriveDistance(0.8, Units.inchesToMeters(31), m_drivetrain),
      new TurnDegrees(0.8, -20, m_drivetrain),
      new IntakeCommand(0, true, collector),
      new IntakeCommand(-1, 1.5, collector)
    );

    straightCommandGroup = new SequentialCommandGroup(
      new IntakeCommand(1, true, collector),
      new DriveDistance(0.7, Units.inchesToMeters(59), m_drivetrain),
      new TurnDegrees(0.7, 20, m_drivetrain),
      new IntakeCommand(-1, 1.5, collector),
      new IntakeCommand(0, true, collector),
      new TurnDegrees(0.7, -20, m_drivetrain),
      new TurnDegrees(0.6, -90, m_drivetrain),
      new DriveDistance(0.7, Units.inchesToMeters(12), m_drivetrain),
      new TurnDegrees(0.6, 90, m_drivetrain),
      new DriveDistance(0.7, Units.inchesToMeters(11), m_drivetrain)
    );

    m_chooser.setDefaultOption("Straight Alliance Antics Auto", () -> straightCommandGroup);
    m_chooser.addOption("Bounce 1", () -> ramseteCommandForTrajectory(bounceTrajectory1));
    m_chooser.addOption("Frantic Fetch", () -> franticFetchCommandGroup);
    m_chooser.addOption("Bounce 2", () -> ramseteCommandForTrajectory(bounceTrajectory2));
    m_chooser.addOption("Bounce 3", () -> ramseteCommandForTrajectory(bounceTrajectory3));
    m_chooser.addOption("Bounce 4", () -> ramseteCommandForTrajectory(bounceTrajectory4));
    m_chooser.addOption("Forward Trajectory", () -> ramseteCommandForTrajectory(forwardTrajectory));
    m_chooser.addOption("Auto Routine Distance", () -> new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", () -> new AutonomousTime(m_drivetrain));
    m_chooser.addOption("Drive For Distance (75 inches)", () -> new DriveDistance(0.8, Units.inchesToMeters(60), m_drivetrain));
    m_chooser.addOption("Alliance Antics Command Group", () -> allianceAnticsCommandGroup);
    m_chooser.addOption("Collector", () -> new InstantCommand(() -> collector.setSpeed(1), collector));

    SmartDashboard.putData(m_chooser);

    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    SmartDashboard.putNumber("Left Volts", 0);
    SmartDashboard.putNumber("Right Volts", 0);
  }

  private void generateTrajectories() {
    mondrianMadnessTrajectory = trajectoryForPath(
      List.of(
          new Pose2d(0, 0, new Rotation2d(0)),
          new Pose2d(dxy, 0, new Rotation2d(0)),
          new Pose2d(2.2*dxy, 1.15*dxy, new Rotation2d(Math.PI / 2)),
          new Pose2d(1.25*dxy, 2.1*dxy, new Rotation2d(Math.PI)),
          new Pose2d(0.15*dxy, 1.3*dxy, new Rotation2d(Math.PI)),
          new Pose2d(-1*dxy, 2.2*dxy, new Rotation2d(Math.PI / 2)),
          new Pose2d(1.3*dxy, 3.05*dxy, new Rotation2d(-Math.PI / 8))
          //new Pose2d(Units.inchesToMeters(-3.5), Units.inchesToMeters(6.5), new Rotation2d(Math.PI / 2))
        ), 
      false);

    barrelRaceTrajectory = trajectoryForPath(
      List.of(
        new Pose2d(0, 0, new Rotation2d()),
        new Pose2d(4*dxy, 0, new Rotation2d()),
        new Pose2d(5*dxy, -dxy, new Rotation2d(-Math.PI / 2)),
        new Pose2d(3*dxy, -dxy, new Rotation2d((-3*Math.PI) / 2)),
        new Pose2d(5*dxy, 0, new Rotation2d()),
        new Pose2d(7*dxy, -1.5*dxy, new Rotation2d()),
        new Pose2d(7*dxy, 2*dxy, new Rotation2d(Math.PI)),
        new Pose2d(6*dxy, dxy, new Rotation2d((7*Math.PI) / 4)),
        new Pose2d(9*dxy, -2.5*dxy, new Rotation2d()),
        new Pose2d(9*dxy, 0, new Rotation2d((3*Math.PI) / 4)),
        new Pose2d(-dxy, 0, new Rotation2d((3*Math.PI) / 4))
      ), 
    false);

    slalomRaceTrajectory = trajectoryForPath(
      List.of(new Pose2d(0, 0, new Rotation2d()),
        new Pose2d(1.5*dxy, 1.5*dxy, new Rotation2d(Math.PI / 3)),
        new Pose2d(4*dxy, 2*dxy, new Rotation2d(-Math.PI / 6)),
        new Pose2d(7*dxy, dxy, new Rotation2d()),
        new Pose2d(7.5*dxy, dxy, new Rotation2d(-Math.PI / 3)),
        //new Pose2d(9*dxy, 0, new Rotation2d(Math.PI / 6)),
        new Pose2d(10*dxy, 2*dxy, new Rotation2d(Math.PI / 2)),
        new Pose2d(9*dxy, 2.5*dxy, new Rotation2d(Math.PI)),
        new Pose2d(8*dxy, 0, new Rotation2d((7*Math.PI) / 6)),
        new Pose2d(6.5*dxy, 0, new Rotation2d(Math.PI)),
        new Pose2d(3*dxy, 0, new Rotation2d(Math.PI)),
        new Pose2d(1.75*dxy, 1.5*dxy, new Rotation2d((5*Math.PI) / 6)),
        new Pose2d(0, dxy, new Rotation2d(Math.PI))
      ), 
      false);

    bounceTrajectory1 = trajectoryForPath(
      List.of(new Pose2d(0, 0, new Rotation2d()),
        new Pose2d(1.5*dxy, 2*dxy, new Rotation2d(2*Math.PI / 3))
      ), 
      false);
    
    bounceTrajectory2 = trajectoryForPath(
      List.of(
        new Pose2d(0, 0, new Rotation2d(Math.PI)),
        new Pose2d(1.25*dxy, 0.5*dxy, new Rotation2d(-5*Math.PI / 6)),
        new Pose2d(3*dxy, 1.6*dxy, new Rotation2d(-Math.PI / 2)),
        new Pose2d(2*dxy, 2.75*dxy, new Rotation2d()),
        new Pose2d(-0.15*dxy, 2.75*dxy, new Rotation2d())
      ), 
      true);

    bounceTrajectory3 = trajectoryForPath(
      List.of(
        new Pose2d(0, 0, new Rotation2d()),
        new Pose2d(2.5*dxy, 0, new Rotation2d()),
        new Pose2d(3*dxy, 1.5*dxy, new Rotation2d(Math.PI / 2)),
        new Pose2d(2*dxy, 2.75*dxy, new Rotation2d(Math.PI)),
        new Pose2d(-0.15*dxy, 3*dxy, new Rotation2d((5*Math.PI) / 6))
      ), 
      false);

    bounceTrajectory4 = trajectoryForPath(
      List.of(
        new Pose2d(0, 0, new Rotation2d(Math.PI)),
        new Pose2d(0.75*dxy, 2*dxy, new Rotation2d(-Math.PI / 2))
      ), 
      true);

    forwardTrajectory = trajectoryForPath(
      List.of(
        new Pose2d(),
        new Pose2d(Units.inchesToMeters(75), 0, new Rotation2d())
      ), 
      false);
  }

  public void updateDashboard() {
    m_drivetrain.updateDashboard();
  }

  private Trajectory trajectoryForPath(List<Pose2d> path, boolean reversed) {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts, 
                                       DriveConstants.kvVoltSecondsPerMeter, 
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint)
            .setReversed(reversed);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      path,
      config);

      return trajectory;
  }

  private Command ramseteCommandForTrajectory(Trajectory trajectory) {
    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        m_drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        m_drivetrain.getFeedforward(),
        m_drivetrain.getKinematics(),
        m_drivetrain::getWheelSpeeds,
        m_drivetrain.getLeftPIDController(),
        m_drivetrain.getRightPIDController(),
        m_drivetrain::tankDriveVolts,
        m_drivetrain);

    //m_drivetrain.plotTrajectory(trajectory);

    m_drivetrain.resetOdometry(trajectory.getInitialPose());
    return ramseteCommand.andThen(() -> m_drivetrain.arcadeDrive(0, 0));
  }

  private Command ramseteCommandForPath(List<Pose2d> path, boolean reversed) {
    Trajectory trajectory = trajectoryForPath(path, reversed);
    Command ramseteCommand = ramseteCommandForTrajectory(trajectory);

    // Set up a sequence of commands
    // First, we want to reset the drivetrain odometry
    return new InstantCommand(() -> m_drivetrain.resetOdometry(trajectory.getInitialPose()), m_drivetrain)
        // next, we run the actual ramsete command
        .andThen(ramseteCommand)
        // Finally, we make sure that the robot stops
        .andThen(new InstantCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain));
  } 

  private Command immediateRamseteCommand(List<Pose2d> path, boolean reversed) {
    Trajectory trajectory = trajectoryForPath(path, reversed);
    Command ramseteCommand = ramseteCommandForTrajectory(trajectory);

    // Since this method is meant to be used for generating continuous drive control paths,
    // we don't interrupt/reset the odometry or stop the motors at the end of the path. 
    return ramseteCommand;
  }

  public void testPeriodic() {
    Pose2d pose = m_drivetrain.getPose();
    System.out.println("new Pose2d("+pose.getX()+", "+pose.getY()+", new Rotation2d("+pose.getRotation().getRadians()+")),");
    System.out.println("L: "+m_drivetrain.getLeftEncoderCount()+", R: "+m_drivetrain.getRightEncoderCount());
  }

  // Ramsete Drive Control - Drive by continuously navigating to a point
  // that is a short distance ahead of our current location:

  // Manage preemptive command-based drive control:

  private Command m_driveCommand = null;

  void drive(Command driveCommand) {
    // Preempt any existing command:
    if (m_driveCommand != null) {
      m_driveCommand.cancel();
    }
    // Schedule and take note of the new drive command:
    m_driveCommand = driveCommand;
    if (m_driveCommand != null) {
      m_driveCommand.schedule();
    } else {
      // No command? then stop.
      new InstantCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain).schedule();
    }
  }

  // Generate and immediately execute a short Ramsete trajectory:
  private void translateDrive(double distance, double angle) {
    boolean reversed = (distance < 0.0)? true : false;
    Translation2d vector = new Translation2d(distance, 0.0).rotateBy(new Rotation2d(angle));
    Transform2d delta = new Transform2d(vector, new Rotation2d(angle));
    Pose2d here = m_drivetrain.getPose();
    Pose2d there = here.plus(delta);

    System.out.println("translateDrive("+distance+", "+angle+")");
    System.out.print(here);
    System.out.print(there);
    System.out.println();

    drive(immediateRamseteCommand(
        List.of(
        here,
        there
      ),
      reversed));
  }

  // Halt any drive command that may be running:
  @SuppressWarnings("unused")
  private void translateDriveHalt() {
    drive(null);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drBumpRight.whenPressed(new InstantCommand(() -> collector.setSpeed(1), collector));
    drBumpRight.whenReleased(new InstantCommand(() -> collector.setSpeed(0), collector));

    drBumpLeft.whenPressed(new InstantCommand(() -> collector.setSpeed(-1), collector));
    drBumpLeft.whenReleased(new InstantCommand(() -> collector.setSpeed(0), collector));
  }

  public void flipTeleOpDriveSide() {
    teleopDriveSide = teleopDriveSide == 1 ? -1 : 1;
  }

  public int getTeleOpDriveSide() {
    return teleopDriveSide;
  }

  public Drivetrain getDrive() {
    return m_drivetrain;
  }

  public void disabledInit() {
    collector.setSpeed(0);
  }

  public void teleopInit() {
    collector.setSpeed(0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected().get();
    //return ramseteCommandForTrajectory(slalomRaceTrajectory);
    //return franticFetchCommandGroup;
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_controller.getRawAxis(1), () -> m_controller.getRawAxis(4),
        () -> m_controller.getRawAxis(3), () -> m_controller.getRawAxis(2),
        () -> m_controller.getBButtonPressed());
  }
}
