package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    // ── Four MK4i modules (FL, FR, BL, BR) ───────────────────────────────────
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.FL_DRIVE_ID, DriveConstants.FL_TURN_ID,
            DriveConstants.FL_DRIVE_INVERTED);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.FR_DRIVE_ID, DriveConstants.FR_TURN_ID,
            DriveConstants.FR_DRIVE_INVERTED);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.BL_DRIVE_ID, DriveConstants.BL_TURN_ID,
            DriveConstants.BL_DRIVE_INVERTED);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.BR_DRIVE_ID, DriveConstants.BR_TURN_ID,
            DriveConstants.BR_DRIVE_INVERTED);

    // ── Pigeon 2 mounted at center of frame ───────────────────────────────────
    private final Pigeon2 pigeon = new Pigeon2(DriveConstants.PIGEON_ID);

    // ── Pose estimation ───────────────────────────────────────────────────────
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field = new Field2d();

    public DriveSubsystem() {
        pigeon.reset();
        SmartDashboard.putData("Field", field);

        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.KINEMATICS,
                getHeading(),
                getModulePositions(),
                new Pose2d());
    }

    @Override
    public void periodic() {
        poseEstimator.update(getHeading(), getModulePositions());
        field.setRobotPose(getPose());

        SmartDashboard.putNumber("Heading (deg)", getHeading().getDegrees());
        SmartDashboard.putNumber("Pitch  (deg)",  pigeon.getPitch().getValueAsDouble());
        SmartDashboard.putNumber("Roll   (deg)",  pigeon.getRoll().getValueAsDouble());
        SmartDashboard.putNumber("Pose X (m)",    getPose().getX());
        SmartDashboard.putNumber("Pose Y (m)",    getPose().getY());
    }

    // ── Heading ───────────────────────────────────────────────────────────────

    /** Returns robot heading as Rotation2d (CCW positive, WPILib convention). */
    public Rotation2d getHeading() {
        // Pigeon 2 is CW-positive; negate for WPILib CCW-positive convention
        return Rotation2d.fromDegrees(-pigeon.getYaw().getValueAsDouble());
    }

    /** Zeros the gyro so the current direction becomes "forward." */
    public void zeroHeading() {
        pigeon.reset();
    }

    // ── Pose ──────────────────────────────────────────────────────────────────

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    }

    // ── Driving ───────────────────────────────────────────────────────────────

    /**
     * Drives the robot.
     *
     * @param xSpeed        Forward speed in m/s (positive = away from driver station)
     * @param ySpeed        Sideways speed in m/s (positive = left)
     * @param rot           Angular speed in rad/s (positive = CCW)
     * @param fieldRelative True for field-relative control, false for robot-relative
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
                : new ChassisSpeeds(xSpeed, ySpeed, rot);

        setModuleStates(DriveConstants.KINEMATICS.toSwerveModuleStates(speeds));
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_DRIVE_SPEED_MPS);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    /** Returns the measured chassis speeds from the module states. */
    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.KINEMATICS.toChassisSpeeds(
                frontLeft.getState(),  frontRight.getState(),
                backLeft.getState(),   backRight.getState());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /** Zeros all drive + turn encoders. Call before enabling if no absolute encoders. */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }
}
