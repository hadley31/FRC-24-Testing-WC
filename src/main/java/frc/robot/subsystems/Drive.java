package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Drive extends SubsystemBase {
  private static final double kGearRatio = 12.4;
  private static final double kWheelDiameter = Units.inchesToMeters(6.0 * 1.09);
  private static final double kWheelRadius = kWheelDiameter / 2;
  private static final double kWheelCircumference = Math.PI * kWheelDiameter;
  private static final double kTrackWidth = Units.inchesToMeters(23.5);

  private static final double kMaxLinearVelocity = Units.feetToMeters(20);
  private static final double kMaxAngularVelocity = Units.degreesToRadians(720);

  StructPublisher<ChassisSpeeds> m_currentSpeedsPub = NetworkTableInstance.getDefault()
      .getStructTopic("/Drive/CurrentSpeeds", ChassisSpeeds.struct).publish();
  StructPublisher<DifferentialDriveWheelSpeeds> m_desiredWheelSpeedsPub = NetworkTableInstance.getDefault()
      .getStructTopic("/Drive/DesiredSpeeds", DifferentialDriveWheelSpeeds.struct).publish();
  StructPublisher<DifferentialDriveWheelVoltages> m_desiredWheelVoltagesPub = NetworkTableInstance.getDefault()
      .getStructTopic("/Drive/DesiredVoltages", DifferentialDriveWheelVoltages.struct).publish();
  StructPublisher<Pose2d> m_currentPosePub = NetworkTableInstance.getDefault()
      .getStructTopic("/Drive/CurrentPose", Pose2d.struct).publish();
  DoublePublisher m_leftDistancePub = NetworkTableInstance.getDefault().getDoubleTopic("/Drive/LeftDistance")
      .publish();
  DoublePublisher m_leftSpeedPub = NetworkTableInstance.getDefault().getDoubleTopic("/Drive/LeftSpeed").publish();
  DoublePublisher m_rightDistancePub = NetworkTableInstance.getDefault().getDoubleTopic("/Drive/RightDistance")
      .publish();
  DoublePublisher m_rightSpeedPub = NetworkTableInstance.getDefault().getDoubleTopic("/Drive/RightSpeed").publish();

  private final CANSparkMax m_leftLeader;
  private final CANSparkMax m_leftFollower;
  private final CANSparkMax m_rightLeader;
  private final CANSparkMax m_rightFollower;
  private final AHRS m_gyro;

  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  private final PIDController m_leftSpeedController = new PIDController(1.8, 0, 0);
  private final PIDController m_rightSpeedController = new PIDController(1.8, 0, 0);

  private final Field2d m_field = new Field2d();

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);
  private final DifferentialDrivePoseEstimator m_estimator;
  private final DifferentialDrivetrainSim m_sim;

  private final DifferentialDriveWheelSpeeds m_targetWheelSpeeds = new DifferentialDriveWheelSpeeds();
  private final DifferentialDriveWheelVoltages m_targetWheelVoltages = new DifferentialDriveWheelVoltages();

  // private final DifferentialDrive m_drive;

  public Drive(int leaderLeftPort, int followerLeftPort, int leaderRightPort, int followerRightPort) {
    m_leftLeader = new CANSparkMax(leaderLeftPort, CANSparkLowLevel.MotorType.kBrushless);
    m_leftFollower = new CANSparkMax(followerLeftPort, CANSparkLowLevel.MotorType.kBrushless);
    m_rightLeader = new CANSparkMax(leaderRightPort, CANSparkLowLevel.MotorType.kBrushless);
    m_rightFollower = new CANSparkMax(followerRightPort, CANSparkLowLevel.MotorType.kBrushless);
    m_gyro = new AHRS(Port.kMXP);

    m_leftLeader.setInverted(true);

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    m_leftEncoder = m_leftLeader.getEncoder();
    m_rightEncoder = m_rightLeader.getEncoder();

    m_leftEncoder.setPositionConversionFactor(kWheelCircumference / kGearRatio);
    m_leftEncoder.setVelocityConversionFactor(kWheelCircumference / kGearRatio / 60);

    m_rightEncoder.setPositionConversionFactor(kWheelCircumference / kGearRatio);
    m_rightEncoder.setVelocityConversionFactor(kWheelCircumference / kGearRatio / 60);

    m_estimator = new DifferentialDrivePoseEstimator(m_kinematics, getGyroAngle(), getLeftDistanceMeters(),
        getRightDistanceMeters(), new Pose2d(2, 2, Rotation2d.fromDegrees(0)));

    SmartDashboard.putData("Field", m_field);

    // m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);

    m_sim = Robot.isSimulation() ? initSim() : null;
  }

  @Override
  public void periodic() {
    var voltages = getCalculatedInputVoltages(m_targetWheelSpeeds);
    m_targetWheelVoltages.left = voltages.left;
    m_targetWheelVoltages.right = voltages.right;

    m_leftLeader.setVoltage(m_targetWheelVoltages.left);
    m_rightLeader.setVoltage(m_targetWheelVoltages.right);

    m_estimator.update(getGyroAngle(), getWheelPositions());

    m_currentSpeedsPub.accept(getChassisSpeeds());
    m_currentPosePub.accept(getPose2d());
    m_leftDistancePub.accept(getLeftDistanceMeters());
    m_leftSpeedPub.accept(getLeftSpeedMetersPerSecond());
    m_rightDistancePub.accept(getRightDistanceMeters());
    m_rightSpeedPub.accept(getRightSpeedMetersPerSecond());
    m_desiredWheelSpeedsPub.accept(m_targetWheelSpeeds);
    m_desiredWheelVoltagesPub.accept(m_targetWheelVoltages);

    SmartDashboard.putNumber("Gyro Angle", getGyroAngle().getDegrees());
    m_field.setRobotPose(getPose2d());
  }

  @Override
  public void simulationPeriodic() {
    m_sim.setInputs(m_targetWheelVoltages.left, m_targetWheelVoltages.right);
    m_sim.update(0.02);

    m_gyro.setAngleAdjustment(-m_sim.getHeading().getDegrees());

    m_leftEncoder.setPosition(m_sim.getLeftPositionMeters());
    m_rightEncoder.setPosition(m_sim.getRightPositionMeters());
    var simSpeeds = new DifferentialDriveWheelSpeeds(m_sim.getLeftVelocityMetersPerSecond(),
        m_sim.getRightVelocityMetersPerSecond());

    m_currentSpeedsPub.accept(m_kinematics.toChassisSpeeds(simSpeeds));
  }

  public Rotation2d getGyroAngle() {
    return m_gyro.getRotation2d();
  }

  public double getLeftDistanceMeters() {
    return m_leftEncoder.getPosition();
  }

  public double getRightDistanceMeters() {
    return m_rightEncoder.getPosition();
  }

  public double getLeftSpeedMetersPerSecond() {
    return m_leftEncoder.getVelocity();
  }

  public double getRightSpeedMetersPerSecond() {
    return m_rightEncoder.getVelocity();
  }

  public DifferentialDriveWheelPositions getWheelPositions() {
    return new DifferentialDriveWheelPositions(getLeftDistanceMeters(), getRightDistanceMeters());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeedMetersPerSecond(), getRightSpeedMetersPerSecond());
  }

  public Pose2d getPose2d() {
    return m_estimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    m_estimator.resetPosition(getGyroAngle(), getLeftDistanceMeters(), getRightDistanceMeters(), pose);
  }

  private DifferentialDriveWheelVoltages getCalculatedInputVoltages(DifferentialDriveWheelSpeeds speeds) {
    double leftVoltage = m_leftSpeedController.calculate(getLeftSpeedMetersPerSecond(), speeds.leftMetersPerSecond);
    double rightVoltage = m_rightSpeedController.calculate(getRightSpeedMetersPerSecond(),
        speeds.rightMetersPerSecond);
    return new DifferentialDriveWheelVoltages(leftVoltage, rightVoltage);
  }

  // public void drive(double forward, double turn) {
  //   m_drive.arcadeDrive(forward, turn);
  // }

  public void drive(DifferentialDriveWheelSpeeds speeds) {
    m_targetWheelSpeeds.leftMetersPerSecond = speeds.leftMetersPerSecond;
    m_targetWheelSpeeds.rightMetersPerSecond = speeds.rightMetersPerSecond;
  }

  public void drive(ChassisSpeeds speeds) {
    drive(m_kinematics.toWheelSpeeds(speeds));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public Command driveCommand(Supplier<Double> forwardInput, Supplier<Double> turnInput) {
    return run(() -> drive(
        new ChassisSpeeds(forwardInput.get() * kMaxLinearVelocity, 0, turnInput.get() * kMaxAngularVelocity)));
    // return run(() -> drive(forwardInput.get(), turnInput.get()));
  }

  private DifferentialDrivetrainSim initSim() {
    var xStd = 0.001;
    var yStd = 0.001;
    var headingStd = 0.001;
    var leftVelStd = 0.1;
    var rightVelStd = 0.1;
    var leftPosStd = 0.005;
    var rightPosStd = 0.005;

    var std = VecBuilder.fill(xStd, yStd, headingStd, leftVelStd, rightVelStd, leftPosStd, rightPosStd);

    return new DifferentialDrivetrainSim(
        DCMotor.getNEO(2),
        kGearRatio,
        0.1,
        Units.lbsToKilograms(20),
        kWheelRadius,
        kTrackWidth,
        std);
  }
}
