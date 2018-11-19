package frc.robot.subsystems;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Logger extends Subsystem {

    private static final Logger M_LOGGER = new Logger();

    private File base;
    private PrintWriter printWriter;
    private List<String> numberKeys, stringKeys;
    private String toWrite;
    private boolean initSuccess;

    private Logger(){
        numberKeys = new ArrayList<>();
        stringKeys = new ArrayList<>();
        try {
            base = getMount();
            System.out.println(base.getAbsolutePath());
            printWriter = new PrintWriter(new BufferedWriter(new FileWriter(base)));
            initSuccess = true;
        } catch (Exception e) {
            DriverStation.reportError("Failed to initialize log on file!", false);
        }
    }

    public static Logger getInstance(){
        return M_LOGGER;
    }

    public void addNumberKeys(String[] keys){
        Collections.addAll(numberKeys, keys);
    }

    public void addStringKeys(String[] keys){
        Collections.addAll(stringKeys, keys);
    }

    public void outputTelemetry(){
        if (printWriter != null && initSuccess) {
            toWrite = "" + Timer.getFPGATimestamp() + Constants.DATA_SEPERATOR;
            for (String key : numberKeys) {
                toWrite += "" + SmartDashboard.getNumber(key, 0.0) + Constants.DATA_SEPERATOR;
            }
            for (String key : stringKeys) {
                toWrite += "" + SmartDashboard.getNumber(key, 0.0) + Constants.DATA_SEPERATOR;
            }
            toWrite += "\r\n";
            //System.out.println(toWrite);
            printWriter.write(toWrite);
            printWriter.flush();
        }
        else{
            DriverStation.reportWarning("logger called to init on Null file stream", false);
        }
    }

    @Override
    public void stop() {
        // no OP
    }

    @Override
    public void reset() {
        // no OP
    }

    private File getMount() {
        File media = new File("/media");
        File logging_path = null;
        for(File mount : media.listFiles())
        {
            logging_path = new File(mount.getAbsolutePath() + "/logging");
            if(logging_path.isDirectory()) {
                System.out.println(logging_path.getAbsolutePath());
                break;
            }
            logging_path = null;
        }
        if (!logging_path.equals(null)) {
            SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyyMMdd_HHmmss_SSS");
            outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern"));
            String newDateString = outputFormatter.format(new Date());
            // build the new filename
            String fileName = newDateString + "_LOG.csv";
            // build the full file path name
            return new File(logging_path.getAbsolutePath() + File.separator + fileName);
        }
        return null;
    }

}