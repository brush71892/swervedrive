
package frc.robot.things;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

    public int id;
    private Rotation2d angleOffset;

    private TalonFX driveMotor;
    private SparkMax turnMotor;

    private CANcoder angleEncoder;
    private RelativeEncoder turnEncoder;

    private CANcoderConfigurator angleEncoderConfigurator;
    private CANcoderConfiguration angleEncoderConfiguration;
    private EncoderConfig turnEncoderConfig;
    private SparkMaxConfig turnSparkMaxConfig;
    private TalonFXConfiguration driveMotorConfiguration;
    private TalonFXConfigurator driveMotorConfigurator;

    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    public SwerveModule(int id, ModuleConstants moduleConstants) {
        this.id = id;
        this.angleOffset = moduleConstants.angleOffset;

        turnMotor = new SparkMax(moduleConstants.turnMotorID, MotorType.kBrushless);
        configureTurnMotor();

        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configureDriveMotor();

        angleEncoder = new CANcoder(moduleConstants.canCoderID);
        configureEncoders();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState.optimize(getState().angle);
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(getState().angle).getCos();
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double pcOutput = desiredState.speedMetersPerSecond / SwerveConstants.kPhysicalMaxSpeed;
            driveMotor.set(pcOutput);
        }
        else {
            driveVelocity.Velocity = desiredState.speedMetersPerSecond / SwerveConstants.wheelCircumference;
            driveMotor.setControl(driveVelocity);
        }
    }

    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = desiredState.angle;
        SparkClosedLoopController controller = turnMotor.getClosedLoopController();
        controller.setSetpoint(angle.getDegrees(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void reset() {
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        turnEncoder.setPosition(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.driveRevToMeters, getAngle());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(turnEncoder.getPosition());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            driveMotor.getPosition().getValueAsDouble() * SwerveConstants.driveRevToMeters, getAngle()
        );
    }

    public int getModuleID() {
        return id;
    }

    public double getMotorSpeed() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    public void setModuleID(int id) {
        this.id = id;
    } 

    private void configureEncoders() {
        angleEncoderConfigurator = angleEncoder.getConfigurator();
        angleEncoderConfiguration = new CANcoderConfiguration();

        angleEncoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        angleEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        angleEncoderConfigurator.apply(angleEncoderConfiguration);

        turnEncoder = turnMotor.getEncoder();
    }

    private void configureTurnMotor() {
        turnSparkMaxConfig = new SparkMaxConfig();
        turnEncoderConfig = new EncoderConfig();
        turnSparkMaxConfig
                .idleMode(SwerveConstants.angleIdleMode)
                .inverted(SwerveConstants.angleMotorInvert).closedLoop
                .pid(
                        SwerveConstants.turnKP,
                        SwerveConstants.turnKI,
                        SwerveConstants.turnKD,
                        ClosedLoopSlot.kSlot0)
                .outputRange(-SwerveConstants.anglePower, SwerveConstants.anglePower);
        turnSparkMaxConfig.closedLoop.feedForward.kV(SwerveConstants.turnKF);
        turnEncoderConfig
                .positionConversionFactor(SwerveConstants.DegreesPerTurnRotation)
                .velocityConversionFactor(SwerveConstants.DegreesPerTurnRotation / 60); 
        turnSparkMaxConfig.encoder.apply(turnEncoderConfig);
        turnMotor.configure(turnSparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureDriveMotor() {
        driveMotorConfigurator = driveMotor.getConfigurator();
        driveMotorConfiguration = new TalonFXConfiguration();

        driveMotorConfiguration.Slot0.kP = SwerveConstants.driveKP;
        driveMotorConfiguration.Slot0.kI = SwerveConstants.driveKI;
        driveMotorConfiguration.Slot0.kD = SwerveConstants.driveKD;
        driveMotorConfiguration.Slot0.kS = SwerveConstants.driveKS;
        driveMotorConfiguration.Slot0.kA = SwerveConstants.driveKA;
        driveMotorConfiguration.Slot0.kV = SwerveConstants.driveKV;

        driveMotorConfiguration.MotorOutput.Inverted = SwerveConstants.driveMotorInvert;
        driveMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveMotorConfigurator.apply(driveMotorConfiguration);
    }
}