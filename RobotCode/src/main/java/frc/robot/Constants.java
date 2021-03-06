// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ShooterConstants{
        public static final double kP = 0.00005;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kIZone = 0;
        public static final double kFeedForward = 0.00019;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;
        public static final double maxRPM = 5700;

        public static final int SHOOTER_MOTOR_ID = 11;
        public static final double PIDTolerance = 0;
    }

    public static final class IntakeConstants{
        public static final int INTAKE_FORWARD_CHANNEL = 2;
        public static final int INTAKE_REVERSE_CHANNEL = 13;
        public static final int INTAKE_MOTOR_ID = 12;
        public static final int INDEX_MOTOR_ID = 10;
    }

    public static final class ClimbConstants{
        public static final int LEFT_CLIMB_MOTOR_ID = 9;
        public static final int RIGHT_CLIMB_MOTOR_ID = 8;
    }

    public static final class DriveConstants{
        public static final int LEFT_MOTOR_1_ID = 4;
        public static final int LEFT_MOTOR_2_ID = 5;
        public static final int RIGHT_MOTOR_1_ID = 6;
        public static final int RIGHT_MOTOR_2_ID = 7;

        public static final int DRIVE_ENCODER_CPR = 2048;
        public static final double DRIVE_WHEEL_DIAMETER = 4;
        public static final double ENCODER_DISTANCE_PER_PULSE = 
            (DRIVE_WHEEL_DIAMETER * Math.PI) / (double) DRIVE_ENCODER_CPR;
    }
    
    public static final class ShiftConstants{
        public static final int SHIFTER_FORWARD_CHANNEL = 4;            
        public static final int SHIFTER_REVERSE_CHANNEL = 15;
    }

    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int MECH_CONTROLLER_ID = 1;
    
    //public static final int RIGHT_STICK_ID = 1;

}
