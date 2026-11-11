
package frc.robot.things;

import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleConstants {
    public final int driveMotorID;
    public final int turnMotorID;
    public final int canCoderID;
    public final Rotation2d angleOffset;

    public ModuleConstants(int driveMotorID, int turnMotorID, int canCoderID, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.canCoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}