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
import frc.robot.subsystems.DriveIO;

public class Drive extends SubsystemBase {

  private final DriveIO io;
  private final Module[] modules = new Module[4];
  public SwerveDriveOdometry swerveOdometry;
  public SwerveDrivePoseEstimator poseEstimator;
  public AHRS gyro;
  private Rotation2d rawGyroRotation = Rotation2d.kZero;

  public Drive(DriveIO io, ModuleIO frontLeftIO, ModuleIO frontRightIO, ModuleIO backLeftIO, ModuleIO backRightIO) {
    this.io = io;
    swerveModules = new SwerveModule[] {
      new Module(frontLeftIO, 0, Constants.ModConstants.Mod0.constants),
      new Module(frontRightIO, 1, Constants.ModConstants.Mod1.constants),
      new Module(backLeftIO, 2, Constants.ModConstants.Mod2.constants),
      new Module(backRightIO, 3, Constants.ModConstants.Mod3.constants)
    };
  }
}
