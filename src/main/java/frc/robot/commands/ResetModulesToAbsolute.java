package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;

public class ResetModulesToAbsolute extends CommandBase{
    private Swerve s_Swerve;
    public ResetModulesToAbsolute(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }


    public void execute() {
        s_Swerve.resetModulesToAbsolute();
        for(SwerveModule mod : s_Swerve.mSwerveMods){
            System.out.println("Module " + mod + ": " + mod.getCanCoder().getDegrees()); 
        }
    }

    public boolean isFinished() {
        return true;
      }
}