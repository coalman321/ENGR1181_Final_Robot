package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Robot;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.BasicFileAttributes;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.TimeZone;

/**
 * Contains basic functions that are used often.
 */
public class Util {

    public static final double kEpsilon = 1e-9;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static void validateBuild() {
        try {
            //get the path of the currently executing jar file
            String currentJarFilePath = Robot.class.getProtectionDomain().getCodeSource().getLocation().toURI().getPath();
            Path filePath = Paths.get(currentJarFilePath);

            //get file system details from current file
            BasicFileAttributes attr = Files.readAttributes(filePath, BasicFileAttributes.class);
            Date utcFileDate = new Date(attr.lastModifiedTime().toMillis());

            // convert from UTC to local time zone
            SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
            outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern"));
            String newDateString = outputFormatter.format(utcFileDate);

            // write the build date & time to the operator's console log window
            DriverStation.reportWarning("== Robot Name == " + Constants.ROBOT_NAME + "| Build Date and Time: " + newDateString + "|", false);
            if (Constants.ENABLE_MP_TEST_MODE) DriverStation.reportWarning("MP TEST MODE IS ENABLED!", false);
        } catch (
                URISyntaxException e) {
            DriverStation.reportWarning("Error determining filename of current JAR file", true);
            //e.printStackTrace();
        } catch (
                IOException e) {
            DriverStation.reportWarning("General Error trying to determine current JAR file", true);
            //e.printStackTrace();
        }
    }
}
