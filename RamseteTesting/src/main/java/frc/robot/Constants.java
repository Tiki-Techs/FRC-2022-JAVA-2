// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ShiftConstants{
        public static final int SHIFTER_FORWARD_CHANNEL = 6;
        public static final int SHIFTER_REVERSE_CHANNEL = 1;
    }
    public static final class DriveConstants{
        
        public static final int RIGHT_BACK = 4;
        public static final int RIGHT_FRONT = 5;
        public static final int LEFT_FRONT = 6;
        public static final int LEFT_BACK = 7;

        public static final double GEAR_RATIO = 10.71;
        public static final int DRIVE_ENCODER_CPR = 2048;
        public static final double ENCODER_TICKS_PER_ROT = 
            (int) Math.round(DRIVE_ENCODER_CPR * GEAR_RATIO);
        public static final double DRIVE_WHEEL_DIAMETER = 4;
        public static final double ENCODER_DISTANCE_PER_PULSE = 
            (DRIVE_WHEEL_DIAMETER * Math.PI) / (double) DRIVE_ENCODER_CPR;

        public static final double ksVolts = 0.21013;
        public static final double kvVoltSecondsPerMeter = 3.732;
        public static final double kaVoltSecondsSquaredPerMeter = 0.44425;
        public static final double kPDriveVel = 4.7036;

        public static final double kTrackwidthMeters = 0.5842;
        public static final DifferentialDriveKinematics kDriveKinematics = 
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationPerSecondSquared = 3;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

}
