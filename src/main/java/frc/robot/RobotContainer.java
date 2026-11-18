package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SamsDrihCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.sammieswerve.Drive;
import frc.robot.subsystems.sammieswerve.GyroIONavX;
import frc.robot.things.ModuleConstants;
import frc.robot.subsystems.sammieswerve.ModuleIOHardware;
import frc.robot.subsystems.sammieswerve.ModuleIO;
import frc.robot.subsystems.sammieswerve.Module;

public class RobotContainer {
  private SwerveSubsystem driveSubsystem = SwerveSubsystem.getInstance();
  private Drive drive;
  private ModuleConstants moduleConstants;

  private final CommandXboxController driverController = new CommandXboxController(67);

  public RobotContainer() {
    drive = new Drive(
      new GyroIONavX(),
      new ModuleIOHardware(moduleConstants),
      new ModuleIOHardware(moduleConstants),
      new ModuleIOHardware(moduleConstants),
      new ModuleIOHardware(moduleConstants)
    );

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(new SamsDrihCommand(drive, () -> driverController.getLeftY() * 5, () -> driverController.getLeftX() * 5, () -> driverController.getRightX() * 5, () -> true));

    // driveSubsystem.setDefaultCommand(new DriveCommands(() -> driverController.getLeftY() * 5, () -> driverController.getLeftX() * 5, () -> driverController.getRightX() * 5, () -> true));

    driverController.x().onTrue(new InstantCommand(() -> drive.resetGyro(), drive));
  }

  public Command getAutonomousCommand() {
    return Commands.print("brush");
  }
}
