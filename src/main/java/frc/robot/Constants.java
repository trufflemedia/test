package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class DriveConstants {

        // ── Drivetrain geometry (adjust to your actual robot measurements) ──────
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(23.5);
        public static final double WHEEL_BASE_METERS  = Units.inchesToMeters(23.5);

        // ── MK4i wheel ───────────────────────────────────────────────────────────
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE   = Math.PI * WHEEL_DIAMETER_METERS;

        // ── MK4i gear ratios ─────────────────────────────────────────────────────
        // Change DRIVE_GEAR_RATIO to match your module's level:
        //   L1 = 8.14, L2 = 6.75, L3 = 6.12, L4 = 5.14
        public static final double DRIVE_GEAR_RATIO   = 6.75;
        public static final double TURNING_GEAR_RATIO = 150.0 / 7.0; // 21.43:1

        // ── SparkMax encoder conversion factors ──────────────────────────────────
        public static final double DRIVE_ENCODER_POSITION_FACTOR =
                WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;          // meters per motor rotation
        public static final double DRIVE_ENCODER_VELOCITY_FACTOR =
                DRIVE_ENCODER_POSITION_FACTOR / 60.0;            // m/s per RPM

        public static final double TURNING_ENCODER_POSITION_FACTOR =
                (2.0 * Math.PI) / TURNING_GEAR_RATIO;           // radians per motor rotation
        public static final double TURNING_ENCODER_VELOCITY_FACTOR =
                TURNING_ENCODER_POSITION_FACTOR / 60.0;         // rad/s per RPM

        // ── WPILib kinematics (FL, FR, BL, BR order) ─────────────────────────────
        public static final SwerveDriveKinematics KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d( WHEEL_BASE_METERS / 2,  TRACK_WIDTH_METERS / 2),  // FL
                        new Translation2d( WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),  // FR
                        new Translation2d(-WHEEL_BASE_METERS / 2,  TRACK_WIDTH_METERS / 2),  // BL
                        new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2)   // BR
                );

        public static final double MAX_DRIVE_SPEED_MPS    = 4.5;
        public static final double MAX_ANGULAR_SPEED_RADS = 2.0 * Math.PI;

        // ── CAN IDs ───────────────────────────────────────────────────────────────
        public static final int FL_DRIVE_ID = 1;
        public static final int FL_TURN_ID  = 2;
        public static final int FR_DRIVE_ID = 3;
        public static final int FR_TURN_ID  = 4;
        public static final int BL_DRIVE_ID = 5;
        public static final int BL_TURN_ID  = 6;
        public static final int BR_DRIVE_ID = 7;
        public static final int BR_TURN_ID  = 8;
        public static final int PIGEON_ID   = 9;

        // ── Motor inversions (flip if wheels drive in the wrong direction) ────────
        public static final boolean FL_DRIVE_INVERTED   = false;
        public static final boolean FR_DRIVE_INVERTED   = true;
        public static final boolean BL_DRIVE_INVERTED   = false;
        public static final boolean BR_DRIVE_INVERTED   = true;
        public static final boolean TURN_MOTOR_INVERTED = true;

        // ── Current limits ────────────────────────────────────────────────────────
        public static final int DRIVE_CURRENT_LIMIT = 50; // amps
        public static final int TURN_CURRENT_LIMIT  = 30; // amps

        // ── Drive PID (velocity control) ─────────────────────────────────────────
        public static final double DRIVE_KP  = 0.04;
        public static final double DRIVE_KI  = 0.0;
        public static final double DRIVE_KD  = 0.0;
        // FF tuned so full setpoint produces ~1.0 output at NEO free speed (~5880 RPM)
        public static final double NEO_FREE_SPEED_RPM = 5880.0;
        public static final double DRIVE_KFF =
                1.0 / (NEO_FREE_SPEED_RPM * DRIVE_ENCODER_VELOCITY_FACTOR);

        // ── Turn PID (position control with wrapping) ─────────────────────────────
        public static final double TURN_KP = 1.0;
        public static final double TURN_KI = 0.0;
        public static final double TURN_KD = 0.0;
    }

    public static final class OIConstants {
        public static final int    DRIVER_CONTROLLER_PORT = 0;
        public static final double JOYSTICK_DEADBAND      = 0.1;
    }
}
