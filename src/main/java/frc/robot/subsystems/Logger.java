package frc.robot.subsystems;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * LOGGER CONVENTIONS FOR SUBSYSTEMS
 * 1st LOG: ALWAYS SYSTEM TIME
 * 2nd LOG: SUBSYSTEM POSITION (IF APPLICABLE)
 * 3rd LOG: POSITION GOING TO (IF APPLICABLE)
 * 4th LOG: 
 */

public class Logger extends SubsystemBase {
    String subsystemname, strDate, directory = "/home/lvuser/";

    public Logger(String namesubsystem) {
        this.subsystemname = namesubsystem;
    }

    @Override
    public void periodic() {
        Date date = Calendar.getInstance().getTime();
        DateFormat dateFormat = new SimpleDateFormat("yyyy-mm-dd hh:mm:ss"); 
        strDate = dateFormat.format(date);
    }

    public String getSystemTime() {
        return strDate;
    }    

    public String convertToCSV(String[] data) {
        return Stream.of(data)
            .collect(Collectors.joining(","));
    }

    public void initialize() throws IOException {
        File csvOut = new File(directory + subsystemname + "Logs.csv");

        if (!csvOut.exists()) {
            csvOut.createNewFile();
        }
    }

    public void log(ArrayList<String[]> data) throws IOException{
        String filename = directory + subsystemname + "Logs.csv";
        File csvOut = new File(filename);

        try (PrintWriter pw = new PrintWriter(csvOut)) {
            data.stream()
                .map(this::convertToCSV)
                .forEach(pw::println);
        }

        assert csvOut.exists() : "CSV File not found when writing to subsystem " + subsystemname;
    }

    
}
