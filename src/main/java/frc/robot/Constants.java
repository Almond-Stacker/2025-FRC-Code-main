package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int PIGEON_ID = 13;
        public static final String FRC_CANBUS_NAME = "1056_Canivore";

        public static final COTSTalonFXSwerveConstants CHOOSEN_MODULE =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.0); //TODO: This must be tuned to specific robot
        public static final double WHEEL_BASE = Units.inchesToMeters(20.875); //TODO: This must be tuned to specific robot
        public static final double WHEEL_CIRCUMFERENCE = CHOOSEN_MODULE.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue ANGLE_MOTOR_INVERT = CHOOSEN_MODULE.angleMotorInvert;
        public static final InvertedValue DRIVE_MOTOR_INVERT = CHOOSEN_MODULE.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue CANCODER_INVERT = CHOOSEN_MODULE.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CURRENT_LIMIT = 25;
        public static final int ANGLE_CURRENT_THRESHOLD = 40;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_THRESHOLD = true;

        public static final int DRIVE_CURRENT_LIMIT = 35; 
        public static final int DRIVE_CURRENT_THRESHOLD = 60;
         public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1; 
         public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = 1.9;//CHOOSEN_MODULE.angleKP;
        public static final double ANGLE_KI = CHOOSEN_MODULE.angleKI;
        public static final double ANGLE_KD = 0.2;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.12; //TODO: This must be tuned to specific robot
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_KS = 0.32; //TODO: This must be tuned to specific robot
        public static final double DRIVE_KV = 1.51;
        public static final double DRIVE_KA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NETURAL_MODE = NeutralModeValue.Brake;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 1; 
            public static final int ANGLE_MOTOR_ID = 2; 
            public static final int CAN_CODER_ID = 3; 
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-56.865234); 
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 7; 
            public static final int ANGLE_MOTOR_ID = 8; 
            public static final int CANCODER_ID = 9; 
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-100.371094); 
            public static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 4; 
            public static final int ANGLE_MOTOR_ID = 5; 
            public static final int CANCODER_ID = 6; 
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(131.396484);
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 11; 
            public static final int ANGLE_MOTOR_ID = 10; 
            public static final int CANCODER_ID = 12; 
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(80); 
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double K_MAX_SPEED_METERS_PER_SECOND = 3; 
        public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; 
        public static final double K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
         public static final double K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI; 
         public static final double K_P_X_CONTROLLER = 1; 
         public static final double K_P_Y_CONTROLLER = 1; 
         public static final double K_P_THETA_CONTROLLER = 1; /* Constraint for the motion profiled robot angle controller */ 
         public static final TrapezoidProfile.Constraints K_THETA_CONTROLLER_CONSTRAINTS = 
            new TrapezoidProfile.Constraints( K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
    }

    public static final class IntakeSubsystemInfo {
        public static final int FRONT_MOTOR_ID = 3; // PWN 
        public static final int WHEEL_MOTOR_ID = 15; // CANBUS 
        public static final int FLOOR_MOTOR_ID = 14; // CANBUS
    }

    public static final class ShooterSubsystemInfo {
        // arm aiming motors 
        public static final int RIGHT_ARM_MOTOR_ID = 18; // CANBUS 
        public static final int LEFT_ARM_MOTOR_ID = 19; // CANBUS 
        public static final int FEEDER_ENCODER = 1; // DIO 

        // feeding motors 
        public static final int LEFT_FEEDING_MOTOR_ID = 5;
        public static final int RIGHT_FEEDING_MOTOR_ID = 4; 

         // shooting motors
        public static final int TOP_SHOOTING_MOTOR_ID = 20; 
        public static final int BOTTOM_SHOOTING_MOTOR_ID = 21;
    }

    public static final class AmpSubsystemInfo {
        //arm motors 
        public static final int LEFT_ARM_MOTOR_ID = 16; 
        public static final int RIGHT_ARM_MOTOR_ID = 17; 
        // thing that put the note in 
        public static final int AMP_SCORER_MOTOR_ID = 0; // TODO: check if this is right 
        public static final int AMP_ENCODER = 0;
    }
}
