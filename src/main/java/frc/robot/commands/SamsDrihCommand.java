package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.sammieswerve.Drive;

public class SamsDrihCommand extends Command {
  private Drive drih;
  private PIDController rotationController;
  private DoubleSupplier ySpeedSupplier;
  private DoubleSupplier xSpeedSupplier;
  private DoubleSupplier rotationSupplier;
  private BooleanSupplier tip;

  public SamsDrihCommand(Drive drih, DoubleSupplier ySpeedSupplier, DoubleSupplier xSpeedSupplier, DoubleSupplier rotationSupplier, BooleanSupplier tip) {
    this.drih = drih;
    addRequirements(drih);

    rotationController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
    rotationController.enableContinuousInput(-1*Math.PI, Math.PI);
    rotationController.setTolerance(Constants.AutoConstants.kThetaTolerance);

    this.ySpeedSupplier = ySpeedSupplier;
    this.xSpeedSupplier = xSpeedSupplier;
    this.rotationSupplier = rotationSupplier;
    this.tip = tip;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpeed = -1*ySpeedSupplier.getAsDouble();
    double ySpeed = -1*xSpeedSupplier.getAsDouble();
    double rotation = -1*rotationSupplier.getAsDouble();

    xSpeed = Math.abs(xSpeed) > Constants.SwerveConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Constants.SwerveConstants.kDeadband ? ySpeed : 0.0;
    rotation = Math.abs(rotation) > Constants.SwerveConstants.kDeadband ? rotation : 0.0;

    rotation *= Constants.SwerveConstants.kMaxTeleAngularSpeed;
    drih.drive(new Translation2d(xSpeed, ySpeed).times(SwerveConstants.kPhysicalMaxSpeed), rotation, false, true);
  }

  @Override
  public void end(boolean interrupted) {
    drih.drive(new Translation2d(0,0), 0, false, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
