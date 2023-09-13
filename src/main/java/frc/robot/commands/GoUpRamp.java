package frc.robot.commands; //TODO on off switch

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class GoUpRamp extends CommandBase {    
    private Swerve swerve;
    private PIDController linearController;
    private double PIDVal;
    private boolean cancelCommand = false;
    private boolean auto;
    private double balanceThreshold = 2;
    private Timer timer = new Timer();
    int dir;

    public GoUpRamp(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);

        linearController = new PIDController(0.2, 0.0, 0.5);
        linearController.setTolerance(1);
        linearController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        timer.start();
        return;
    }

    public void execute() {
        double pos = swerve.getRoll().getDegrees();
        System.out.println("Activated" + pos);
        PIDVal = linearController.calculate(pos, 21);
        if (pos > -21)
            dir = 1;
        if (pos < -21 || timer.hasElapsed(5)) {
            cancelCommand = true;
        }
        if (MathUtil.clip(PIDVal, -0.5, 0.5) < 0.1
         && auto == true) {
            cancelCommand = true;
        }
        // System.out.println("Activated" + pos);
        System.out.println("wtf: " + MathUtil.clip(PIDVal, -0.4, 0.4));
        this.swerve.drive(
            new Translation2d(Math.abs(MathUtil.clip(PIDVal, -0.4, 0.4)) * dir, 0), 
            0, 
            false, 
            true
        );

        Timer.delay(0.005);	
        }

    public void end(boolean interrupted) {
        cancelCommand = true;
    }
    public boolean isFinished() {
      return cancelCommand;
    }
}