package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.DriveConstants;

/**
 * One MK4i swerve module — NEO v1.1 drive + NEO v1.1 turn, both via SparkMax,
 * with a Redux Helium CANandmag absolute encoder on the turning shaft.
 *
 * The CANandmag seeds the SparkMax relative encoder at startup so wheel angle
 * is always correct regardless of robot orientation when powered on.
 * Zero each encoder's position offset using Redux Alchemist so that
 * 0 rotations = wheel pointing straight forward.
 */
public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final SparkPIDController drivePID;
    private final SparkPIDController turnPID;

    // Absolute encoder — gives true wheel angle even after power cycle
    private final Canandmag canandmag;

    /**
     * @param driveCanId    CAN ID for the drive SparkMax
     * @param turnCanId     CAN ID for the turning SparkMax
     * @param encoderCanId  CAN ID for the Redux Helium CANandmag
     * @param driveInverted True if the drive motor runs the wrong direction
     */
    public SwerveModule(int driveCanId, int turnCanId, int encoderCanId, boolean driveInverted) {
        canandmag  = new Canandmag(encoderCanId);

        driveMotor = new CANSparkMax(driveCanId, MotorType.kBrushless);
        turnMotor  = new CANSparkMax(turnCanId,  MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveInverted);
        turnMotor.setInverted(DriveConstants.TURN_MOTOR_INVERTED);

        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setIdleMode(IdleMode.kBrake);

        driveMotor.setSmartCurrentLimit(DriveConstants.DRIVE_CURRENT_LIMIT);
        turnMotor.setSmartCurrentLimit(DriveConstants.TURN_CURRENT_LIMIT);

        // ── Drive encoder ─────────────────────────────────────────────────────────
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(DriveConstants.DRIVE_ENCODER_POSITION_FACTOR);
        driveEncoder.setVelocityConversionFactor(DriveConstants.DRIVE_ENCODER_VELOCITY_FACTOR);

        // ── Turn encoder (relative, seeded from CANandmag at startup) ─────────────
        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(DriveConstants.TURNING_ENCODER_POSITION_FACTOR);
        turnEncoder.setVelocityConversionFactor(DriveConstants.TURNING_ENCODER_VELOCITY_FACTOR);

        // ── Drive PID (velocity) ──────────────────────────────────────────────────
        drivePID = driveMotor.getPIDController();
        drivePID.setFeedbackDevice(driveEncoder);
        drivePID.setP(DriveConstants.DRIVE_KP);
        drivePID.setI(DriveConstants.DRIVE_KI);
        drivePID.setD(DriveConstants.DRIVE_KD);
        drivePID.setFF(DriveConstants.DRIVE_KFF);
        drivePID.setOutputRange(-1.0, 1.0);

        // ── Turn PID (position with continuous wrap -π to π) ─────────────────────
        turnPID = turnMotor.getPIDController();
        turnPID.setFeedbackDevice(turnEncoder);
        turnPID.setP(DriveConstants.TURN_KP);
        turnPID.setI(DriveConstants.TURN_KI);
        turnPID.setD(DriveConstants.TURN_KD);
        turnPID.setOutputRange(-1.0, 1.0);
        turnPID.setPositionPIDWrappingEnabled(true);
        turnPID.setPositionPIDWrappingMinInput(-Math.PI);
        turnPID.setPositionPIDWrappingMaxInput(Math.PI);

        driveMotor.burnFlash();
        turnMotor.burnFlash();

        // Seed turn encoder from absolute position so angle is correct at startup
        driveEncoder.setPosition(0.0);
        seedTurnEncoderFromAbsolute();
    }

    /**
     * Reads the CANandmag and writes the result into the SparkMax relative
     * encoder so closed-loop control starts from the correct angle.
     * getAbsPosition() returns 0.0–1.0 (one full rotation); we convert to
     * radians and normalize to [-π, π].
     */
    public void seedTurnEncoderFromAbsolute() {
        double absRadians = MathUtil.angleModulus(canandmag.getAbsPosition() * 2.0 * Math.PI);
        turnEncoder.setPosition(absRadians);
    }

    /** Zeros the drive encoder and re-seeds the turn encoder from the absolute encoder. */
    public void resetEncoders() {
        driveEncoder.setPosition(0.0);
        seedTurnEncoderFromAbsolute();
    }

    /** Returns the raw absolute position from the CANandmag in radians [-π, π]. */
    public double getAbsoluteAngleRadians() {
        return MathUtil.angleModulus(canandmag.getAbsPosition() * 2.0 * Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveEncoder.getVelocity(),
                new Rotation2d(turnEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                new Rotation2d(turnEncoder.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentAngle = new Rotation2d(turnEncoder.getPosition());

        // Flip drive direction instead of rotating more than 90° when possible
        SwerveModuleState optimized = SwerveModuleState.optimize(desiredState, currentAngle);

        // Scale drive speed by cosine of angle error to reduce wheel scrub while turning
        optimized.speedMetersPerSecond *= optimized.angle.minus(currentAngle).getCos();

        drivePID.setReference(optimized.speedMetersPerSecond, ControlType.kVelocity);
        turnPID.setReference(optimized.angle.getRadians(),    ControlType.kPosition);
    }

    public void stop() {
        driveMotor.set(0.0);
        turnMotor.set(0.0);
    }
}
