package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.things.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

  public static SwerveSubsystem instance;
  public SwerveModule[] swerveModules;
  public SwerveDriveOdometry swerveOdometry;

  public SwerveDrivePoseEstimator poseEstimator;
  
  private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4];

  public SwerveSubsystem() {

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, Constants.ModConstants.Mod0.constants),
      new SwerveModule(1, Constants.ModConstants.Mod1.constants),
      new SwerveModule(2, Constants.ModConstants.Mod2.constants),
      new SwerveModule(3, Constants.ModConstants.Mod3.constants)
    };

    resetModules();
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = 
      Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds (
          translation.getX(),
          translation.getY(),
          rotation,
          rawGyroRotation
        )
        : new ChassisSpeeds(
          translation.getX(),
          translation.getY(),
          rotation,
          rawGyroRotation
        )
      );

    for (SwerveModule module : swerveModules) {
      module.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public static SwerveSubsystem getInstance() {
    if (instance == null) {
      instance = new SwerveSubsystem();
    }
    return instance;
  }

  private void resetModules() {
    for (SwerveModule module : swerveModules) {
      module.reset();
    }
  }

  @Override
  public void periodic() {}
}
