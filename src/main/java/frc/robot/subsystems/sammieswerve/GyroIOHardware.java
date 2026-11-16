package frc.robot.subsystems.sammieswerve;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class GyroIOHardware implements GyroIO{
    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI, (byte) 0.02);

    public GyroIOHardware() {}

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(-navx.getYaw());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navx.getRawGyroZ());
    }

    @Override
    public void setYaw(Rotation2d angle) {
        navx.setAngleAdjustment(angle.getDegrees());
    }
}
