package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Default teleop drive command — left stick translates, right stick rotates. */
public class DriveCommand extends Command {

    private final DriveSubsystem drive;
    private final XboxController controller;

    // Slew rate limiters smooth joystick inputs and prevent wheel scrub on fast inputs
    private final SlewRateLimiter xLimiter   = new SlewRateLimiter(3.0); // m/s²
    private final SlewRateLimiter yLimiter   = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0); // rad/s²

    public DriveCommand(DriveSubsystem drive, XboxController controller) {
        this.drive      = drive;
        this.controller = controller;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // Negate Y-axes: joystick up is negative in WPILib
        double xSpeed   = applyDeadband(-controller.getLeftY());
        double ySpeed   = applyDeadband(-controller.getLeftX());
        double rotation = applyDeadband(-controller.getRightX());

        xSpeed   = xLimiter.calculate(xSpeed)    * DriveConstants.MAX_DRIVE_SPEED_MPS;
        ySpeed   = yLimiter.calculate(ySpeed)     * DriveConstants.MAX_DRIVE_SPEED_MPS;
        rotation = rotLimiter.calculate(rotation) * DriveConstants.MAX_ANGULAR_SPEED_RADS;

        // Field-relative by default; hold left bumper for robot-relative
        boolean fieldRelative = !controller.getLeftBumper();
        drive.drive(xSpeed, ySpeed, rotation, fieldRelative);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double applyDeadband(double value) {
        return Math.abs(value) > OIConstants.JOYSTICK_DEADBAND ? value : 0.0;
    }
}
