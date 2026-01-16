package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class AlignCommand extends Command {
    private final PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    private final double fixedTurnSpeed = 0.35;
    private final double fixedForwardSpeed = 0.3;

    private final double desiredAngle = 0.0;
    private final double desiredRange = 0.1;

    private final double yawTolerance = 2.0;
    private final double rangeTolerance = 1;

    private double targetYaw = 0.0;
    private double targetRange = 0.0;

    private final SlewRateLimiter yawLimiter = new SlewRateLimiter(7.5);
    private double smoothedYaw = 0.0;
    private boolean targetVisible = false;

    private final SwerveRequest.RobotCentric swerveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain mSwerve;

    public AlignCommand(CommandSwerveDrivetrain mSwerve) {
        this.mSwerve = mSwerve;
    }

    @Override
    public void execute() {

        var results = camera.getLatestResult();
        targetVisible = false;

        if (results.hasTargets()) {
            for (var target : results.getTargets()) {
                if (target.getFiducialId() == 0) {
                    targetYaw = target.getYaw();
                    targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                            0.48, 0.5, Degrees.of(5).in(Radians), Math.toRadians(target.getPitch()));
                    targetVisible = true;
                }
            }
        } 
        else {
            targetVisible = false;
            mSwerve.setControl(swerveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        }

        if (!targetVisible) {
            return;
        }

        smoothedYaw = yawLimiter.calculate(targetYaw);

        double yawError = desiredAngle - smoothedYaw;
        double turn = Math.abs(yawError) < yawTolerance ? 0.0 : (yawError > 0 ? fixedTurnSpeed : -fixedTurnSpeed);

        double rangeError = desiredRange - targetRange;
        double forward = Math.abs(rangeError) < rangeTolerance ? 0.0
                : (rangeError > 0 ? fixedForwardSpeed : -fixedForwardSpeed);

        mSwerve.setControl(
                swerveRequest
                        .withVelocityX(forward)
                        .withRotationalRate(turn));

        SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
        SmartDashboard.putNumber("Vision Target Range (m)", targetRange);
        SmartDashboard.putNumber("Vision Target Yaw (deg)", targetYaw);
        SmartDashboard.putNumber("Smoothed Yaw", smoothedYaw);
        SmartDashboard.putNumber("Turn Command", turn);
        SmartDashboard.putNumber("Forward Command", forward);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
