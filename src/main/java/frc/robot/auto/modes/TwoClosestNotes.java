/*  package frc.robot.auto.modes;

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


public class TwoClosestNotes implements IAuto {
    private final Trajectory m1, m2, m3, m4; 
    private final Pose2d mStartingPose; 
    private final Command mAutoCommand; 
    public TwoClosestNotes () {
        m1 = AutoTrajectoryReader.generateTrajectoryFromFile("PATHS/1.path", Constants.createTrajConfig(2, 2));
        m2 = AutoTrajectoryReader.generateTrajectoryFromFile("PATHS/2.path", Constants.createTrajConfig(2, 2)); 
        m3 = AutoTrajectoryReader.generateTrajectoryFromFile("PATHS/3.path", Constants.createTrajConfig(2, 2)); 
        m4 = AutoTrajectoryReader.generateTrajectoryFromFile("PATHS/4.path", Constants.createTrajConfig(2, 2)); 
        mStartingPose = new Pose2d(m1.getInitialPose().getTranslation(), Rotation2d.fromDegrees(0)); 
        mAutoCommand = new SequentialCommandGroup(
           
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new FollowPath(m1, Rotation2d.fromDegrees(0)),
                    new FollowPath(m2, Rotation2d.fromDegrees(0)),
                    new FollowPath(m3, Rotation2d.fromDegrees(0)),
                    new FollowPath(m4, Rotation2d.fromDegrees(0))
                )
               
            )
           
            new ParallelCommandGroup(
                new SequentialCommandGroup (
                    new FollowPath(m3, Rotation2d.fromDegrees(0)),
                    new FollowPath(m4, Rotation2d.fromDegrees(0))
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
*/