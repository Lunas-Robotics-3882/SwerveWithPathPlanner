package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.Timer;


public class ClimbCommand extends Command{

    private final Climber climber;
    private final LED led;    
    private boolean firstcheck = true;


    public ClimbCommand(Climber climber, LED led) {
        this.climber = climber;
        this.led = led;
     
        
        addRequirements(climber,led);

      }

      @Override
  public void initialize() {
    climber.slowDown();
  }

  @Override
  public void execute()
  {
    climber.slowDown();
  }

      @Override
      public boolean isFinished() {
          return !climber.heightCheck();
      }

     @Override
     public void end(boolean interrupted) {
      climber.setHoldPosition(climber.getEncoder());
      led.DGREEN();
      
    }
}