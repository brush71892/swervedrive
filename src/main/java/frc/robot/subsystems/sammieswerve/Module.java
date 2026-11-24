
package frc.robot.subsystems.sammieswerve;

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
import frc.robot.subsystems.sammieswerve.ModuleIO.ModuleIOInputs;
import frc.robot.things.ModuleConstants;

public class Module {

    public int id;
    private Rotation2d angleOffset;

    private ModuleIO io;
    private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    public Module(ModuleIO io, int id, ModuleConstants moduleConstants) {
        this.io = io;
        this.id = id;
        this.angleOffset = moduleConstants.angleOffset;
    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState.optimize(inputs.state.angle);
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(inputs.state.angle).getCos();
        io.setAngle(desiredState);
        io.setSpeed(desiredState, isOpenLoop);
    }

    public int getModuleID() {
        return id;
    }

    public void setModuleID(int id) {
        this.id = id;
    }

    public SwerveModuleState getState() {
        return inputs.state;
    }

    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    public SwerveModulePosition getModulePosition() {
        return inputs.drivePosition;
    }

}