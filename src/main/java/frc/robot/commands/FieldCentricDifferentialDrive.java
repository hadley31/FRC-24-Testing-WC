package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class FieldCentricDifferentialDrive extends Command {

  private final Drive m_drive;
  private final Supplier<Double> m_xInput;
  private final Supplier<Double> m_yInput;

  private static final double kMaxLinearVelocity = Units.feetToMeters(16);
  private static final double kMaxAngularVelocity = Units.degreesToRadians(1500);

  private Rotation2d m_targetAngle;

  public FieldCentricDifferentialDrive(Supplier<Double> xSupplier, Supplier<Double> ySupplier, Drive drive) {
    m_xInput = xSupplier;
    m_yInput = ySupplier;
    m_drive = drive;
  }

  @Override
  public void initialize() {
    m_targetAngle = m_drive.getPose2d().getRotation();
  }

  @Override
  public void execute() {
    double x = m_xInput.get();
    double y = m_yInput.get();
    double magnitude = Math.hypot(y, x);

    if (Math.abs(magnitude) > 0.1) {
      m_targetAngle = new Rotation2d(x, y);
    }

    var angleDelta = m_targetAngle.minus(m_drive.getPose2d().getRotation());

    double speed = magnitude * kMaxLinearVelocity;
    double angularSpeed = angleDelta.getRadians() / Math.PI * kMaxAngularVelocity;

    if (Math.abs(angleDelta.getDegrees()) > 90) {
      // angularSpeed *= -1;
      speed *= -1;
      angleDelta = angleDelta.minus(Rotation2d.fromDegrees(180));
    }

    if (Math.abs(angleDelta.getDegrees()) > 20) {
      speed /= 10;
    }

    m_drive.drive(new ChassisSpeeds(speed, 0, angularSpeed));
  }
}
