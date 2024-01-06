package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase{
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;

    // Create a NavX gyro over I2C MXP
    private final AHRS m_gyro = new AHRS(I2C.Port.kMXP);

    private ChassisSpeeds speeds = new ChassisSpeeds();

    // Create a swerve drive odometry instance to calculate robot position
    public final SwerveDriveOdometry m_odometry;

    public SwerveDrive(
        SwerveModule m_frontLeft,
        SwerveModule m_frontRight,
        SwerveModule m_backLeft,
        SwerveModule m_backRight
    ) {
        this.m_frontLeft = m_frontLeft;
        this.m_frontRight = m_frontRight;
        this.m_backLeft = m_backLeft;
        this.m_backRight = m_backRight;

        // Reset the gyro to 0 degrees
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                m_gyro.reset();
                Thread.sleep(1000);
                m_gyro.zeroYaw();
            } catch (Exception e) {}
        }).start();

        this.m_odometry = new SwerveDriveOdometry(Constants.Swerve.Physical.m_swerveDriveKinematics, getRotation2d(), new SwerveModulePosition[]{
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.Physical.kMaxSpeedMetersPerSecond);
        m_frontLeft.setState(desiredStates[0]);
        m_frontRight.setState(desiredStates[1]);
        m_backLeft.setState(desiredStates[2]);
        m_backRight.setState(desiredStates[3]);
    }

    public void stopModules() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        this.speeds = speeds;
    }

    public ChassisSpeeds getSpeeds() {
        return this.speeds;
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(Math.toRadians(this.m_gyro.getAngle() % 360));
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };
    }

    public SwerveDriveOdometry getOdometry() {
        try {
            m_odometry.update(getRotation2d(), new SwerveModulePosition[]{
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            });
        } catch (Exception e) {
            System.out.println("Error updating odometry: " + e);
        }
        return m_odometry;
    }
}
