package frc.robot.subsystems;


public class Logger extends Subsystem {

    private static final Logger M_LOGGER = new Logger();

    private Logger(){
        
    }

    public static Logger getInstance(){
        return M_LOGGER;
    }

}