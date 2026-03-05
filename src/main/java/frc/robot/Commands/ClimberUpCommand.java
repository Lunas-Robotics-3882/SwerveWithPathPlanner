package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsytem;
import frc.robot.subsystems.LED;


public class ClimberUpCommand extends Command{

    private final ClimberSubsytem climber;
    private final LED led;    

    public ClimberUpCommand(ClimberSubsytem climber, LED led) {
        this.climber = climber;
        this.led = led;
             
        addRequirements(climber,led);

      }

      @Override
  public void initialize() {
    climber.setPosition(140);
    led.RED();
  }

  @Override
  public void execute()
  {
  }

      @Override
      public boolean isFinished() {
          return !climber.CheckUpPost();
      }

     @Override
     public void end(boolean interrupted) {
      climber.stop();
      led.DGREEN();
      
    }
}