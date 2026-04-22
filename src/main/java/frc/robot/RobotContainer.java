package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    private final XboxController driverController =
            new XboxController(Constants.OIConstants.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        configureButtonBindings();

        // DriveCommand runs whenever no other command needs the drive subsystem
        driveSubsystem.setDefaultCommand(
                new DriveCommand(driveSubsystem, driverController));
    }

    private void configureButtonBindings() {
        // B button — zero the gyro so current direction becomes "forward"
        new JoystickButton(driverController, XboxController.Button.kB.value)
                .onTrue(new InstantCommand(driveSubsystem::zeroHeading, driveSubsystem));

        // Y button — reset odometry to the origin
        new JoystickButton(driverController, XboxController.Button.kY.value)
                .onTrue(new InstantCommand(
                        () -> driveSubsystem.resetPose(new edu.wpi.first.math.geometry.Pose2d()),
                        driveSubsystem));
    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }
}
