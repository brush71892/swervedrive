package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private SwerveSubsystem driveSubsystem = SwerveSubsystem.getInstance();

  private final CommandXboxController driverController = new CommandXboxController(67);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(new DriveCommands(() -> driverController.getLeftY() * 5, () -> driverController.getLeftX() * 5, () -> driverController.getRightX() * 5, () -> true));

    driverController.x().onTrue(new InstantCommand(() -> driveSubsystem.resetGyro(), driveSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
