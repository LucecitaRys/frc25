// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.commands.Autonomos;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.ElevatorSub.ElePoses;
import frc.robot.subsystems.Shooter.intake_states;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Brazo.brazoposes;

/** Add your docs here. */
public class SUPERIOR implements IAuto {
      private final Trajectory mpath1, mpath2, mpath3, mpath4; 
    private final Pose2d mStartingPose; 
    private final Command mAutoCommand; 
  
   
    public SUPERIOR () {
        

        mpath1 = AutoTrajectoryReader.generateTrajectoryFromFile("pathplanner/paths/SUP1.path", Constants.createTrajConfig(2, 2));
        mpath2 = AutoTrajectoryReader.generateTrajectoryFromFile("pathplanner/paths/SUP2.path", Constants.createTrajConfig(2, 2)); 
        mpath3 = AutoTrajectoryReader.generateTrajectoryFromFile("pathplanner/paths/SUP3.path", Constants.createTrajConfig(2, 2)); 
        mpath4 = AutoTrajectoryReader.generateTrajectoryFromFile("pathplanner/paths/SUP4.path", Constants.createTrajConfig(2, 2)); 
        mStartingPose = new Pose2d(mpath1.getInitialPose().getTranslation(), Rotation2d.fromDegrees(0)); 
        
        mAutoCommand = new SequentialCommandGroup(
         
           
                new SequentialCommandGroup(
                    new FollowPath(mpath1, Rotation2d.fromDegrees(0)),
                    new Autonomos(brazoposes.nivel4, intake_states.throwCoral, ElePoses.nivel4),
                    new FollowPath(mpath2, Rotation2d.fromDegrees(0)),
                    new Autonomos(brazoposes.collectCoral, intake_states.collectCoral, ElePoses.collect),
                    new FollowPath(mpath3, Rotation2d.fromDegrees(0)),
                    new Autonomos(brazoposes.nivel3, intake_states.throwCoral, ElePoses.nivel4),
                    new FollowPath(mpath4, Rotation2d.fromDegrees(0)),
                    new Autonomos(brazoposes.collectCoral, intake_states.collectCoral, ElePoses.collect),
                    new FollowPath(mpath3, Rotation2d.fromDegrees(0)),
                    new Autonomos(brazoposes.nivel3, intake_states.throwCoral, ElePoses.nivel3),
                    new FollowPath(mpath4, Rotation2d.fromDegrees(0))
                    
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
