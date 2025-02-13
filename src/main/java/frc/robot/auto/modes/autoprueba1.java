package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup; 
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auto.AutoTrajectoryReader;
import frc.robot.auto.IAuto;

import frc.robot.commands.FollowPath;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.intake_states;




public class autoprueba1 implements IAuto {
    private final Trajectory mpath1, mpath2, mpath3, mpath4; 
    private final Pose2d mStartingPose; 
    private final Command mAutoCommand; 
    private final Shooter mShooter = Shooter.getInstance();

    public autoprueba1 () {
        mpath1 = AutoTrajectoryReader.generateTrajectoryFromFile("pathplanner/paths/test.path", Constants.createTrajConfig(2, 2));
        mpath2 = AutoTrajectoryReader.generateTrajectoryFromFile("pathplanner/paths/test1.path", Constants.createTrajConfig(2, 2)); 
        mpath3 = AutoTrajectoryReader.generateTrajectoryFromFile("pathplanner/paths/test2.path", Constants.createTrajConfig(2, 2)); 
        mpath4 = AutoTrajectoryReader.generateTrajectoryFromFile("pathplanner/paths/test3.path", Constants.createTrajConfig(2, 2)); 
        mStartingPose = new Pose2d(mpath1.getInitialPose().getTranslation(), Rotation2d.fromDegrees(0)); 
        mAutoCommand = new SequentialCommandGroup(
         
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new FollowPath(mpath1, Rotation2d.fromDegrees(0)),
                    new FollowPath(mpath2, Rotation2d.fromDegrees(0))
                )
               
            ),
          
            new ParallelCommandGroup(
                new SequentialCommandGroup (
                    new FollowPath(mpath3, Rotation2d.fromDegrees(0)),
                    new FollowPath(mpath4, Rotation2d.fromDegrees(0))
                )
                
            )
            
        );
    }

    @Override 
    public Command getAutoCommand () {
        return mAutoCommand; 
    }
    @Override
    public Pose2d getStartingPose () {
        return mStartingPose; 
    }
}