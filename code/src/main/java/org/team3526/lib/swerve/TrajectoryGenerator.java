package org.team3526.lib.swerve;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.Swerve.Physical;
import frc.robot.subsystems.Swerve;
public class TrajectoryGenerator {
    public static Trajectory trajectory = new Trajectory();

    public static SwerveControllerCommand newTrajectory(String filePath, Swerve swerve) { // If error try asking for swerve
        try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filePath);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + filePath, ex.getStackTrace());
        }
        
        PIDController xController = new PIDController(1.5, 0, 0);
        PIDController yController = new PIDController(1.5, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0, Physical.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            Physical.kDriveKinematics,
            xController,
            yController,
            thetaController,
            swerve::setModuleStates,
            swerve
        );
         return swerveControllerCommand;
    }
}
