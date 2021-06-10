// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    /*public static final double ksVolts = 0.659;
    public static final double kvVoltSecondsPerMeter = 1.78;
    public static final double kaVoltSecondsSquaredPerMeter = 0.00929;

    public static final double kPDriveVel = 0.0342;

    public static final double kTrackwidthMeters = 0.6603535701153946;*/
    /*public static final double ksVolts = 0.681;
    public static final double kvVoltSecondsPerMeter = 8.02;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0424;

    //public static final double kPDriveVel = 0.0994;
    public static final double kPDriveVel = 0.2;*/

    /*public static final double ksVolts = 1.95;
    public static final double kvVoltSecondsPerMeter = 4.95;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0679;

    public static final double kPDriveVel = 0.528;*/

    /*public static final double ksVolts = 0.929;
    public static final double kvVoltSecondsPerMeter = 6.33;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0389;

    public static final double kPDriveVel = 0.085;*/

    public static final double ksVolts = 1.32;
    public static final double kvVoltSecondsPerMeter = 5.88;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0443;

    public static final double kPDriveVel = 1.5;

    /*public static final double ksVolts = 0.306;
    public static final double kvVoltSecondsPerMeter = 9.6;
    public static final double kaVoltSecondsSquaredPerMeter = 0.111;
6
    public static final double kPDriveVel = 1.86;*/

    /*public static final double ksVolts = 1.32;
    public static final double kvVoltSecondsPerMeter = 6.01;
    public static final double kaVoltSecondsSquaredPerMeter = 0.019;

    public static final double kPDriveVel = 0.00404;*/

    //public static final double kTrackwidthMeters = 0.141;
    public static final double kTrackwidthMeters = 0.6585614249427584;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.6;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    //Command-based PID Controllers
    public static final double kPDrive = 0.12;
    public static final double kPTurn = 0.0095;
  }
}
