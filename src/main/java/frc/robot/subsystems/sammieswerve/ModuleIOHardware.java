package frc.robot.subsystems.sammieswerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.things.ModuleConstants;

public class ModuleIOHardware implements ModuleIO{

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

    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    public ModuleIOHardware(ModuleConstants constants) {

        turnMotor = new SparkMax(constants.turnMotorID, MotorType.kBrushless);
        driveMotor = new TalonFX(constants.driveMotorID);
        angleEncoder = new CANcoder(constants.canCoderID);

        // encoder configs
        angleEncoderConfigurator = angleEncoder.getConfigurator();
        angleEncoderConfiguration = new CANcoderConfiguration();
        angleEncoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        angleEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        angleEncoderConfigurator.apply(angleEncoderConfiguration);
    
        turnEncoder = turnMotor.getEncoder();

        // turn config
        turnSparkMaxConfig = new SparkMaxConfig();
        turnEncoderConfig = new EncoderConfig();
        turnSparkMaxConfig
                .idleMode(SwerveConstants.angleIdleMode)
                .inverted(SwerveConstants.angleMotorInvert).closedLoop
                .pid(
                        SwerveConstants.angleKP,
                        SwerveConstants.angleKI,
                        SwerveConstants.angleKD,
                        ClosedLoopSlot.kSlot0)
                .outputRange(-SwerveConstants.anglePower, SwerveConstants.anglePower);
        turnSparkMaxConfig.closedLoop.feedForward.kV(SwerveConstants.angleKF);
        turnEncoderConfig
                .positionConversionFactor(SwerveConstants.DegreesPerTurnRotation)
                .velocityConversionFactor(SwerveConstants.DegreesPerTurnRotation / 60); 
        turnSparkMaxConfig.encoder.apply(turnEncoderConfig);
        turnMotor.configure(turnSparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        // drive config
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

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.state = new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.driveRevToMeters, getAngle());
        inputs.driveConnected = true;
        inputs.drivePosition = new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble() * SwerveConstants.driveRevToMeters, getAngle());
        inputs.driveVelocityRadPerSec = driveMotor.getVelocity().getValueAsDouble();
        inputs.turnConnected = true;
        inputs.turnEncoderConnected = true;
        inputs.turnPosition = Rotation2d.fromDegrees(turnEncoder.getPosition());
        inputs.turnAbsolutePositionDeg = turnEncoder.getPosition();
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = desiredState.angle;
        SparkClosedLoopController controller = turnMotor.getClosedLoopController();
        controller.setSetpoint(angle.getDegrees(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double pcOutput = desiredState.speedMetersPerSecond / SwerveConstants.kPhysicalMaxSpeed;
            driveMotor.set(pcOutput);
        }
        else {
            driveVelocity.Velocity = desiredState.speedMetersPerSecond / SwerveConstants.wheelCircumference;
            driveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(turnEncoder.getPosition());
    }
}
