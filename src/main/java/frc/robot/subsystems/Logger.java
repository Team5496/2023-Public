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

public class Logger extends SubsystemBase {
    String subsystemname, strDate;

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

    public String escapeSpecialCharacters(String data) {
        String escapedData = data.replaceAll("\\R", " ");
        if (data.contains(",") || data.contains("\"") || data.contains("'")) {
            data = data.replace("\"", "\"\"");
            escapedData = "\"" + data + "\"";
        }
        return escapedData;
    }
    

    public String convertToCSV(String[] data) {
        return Stream.of(data)
            .map(this::escapeSpecialCharacters)
            .collect(Collectors.joining(","));
    }

    public void log(ArrayList<String[]> data) throws IOException {
        String filename = subsystemname + "Logs.csv";
        File csvOut = new File(filename);

        try (PrintWriter pw = new PrintWriter(csvOut)) {
            data.stream()
                .map(this::convertToCSV)
                .forEach(pw::println);
        }

        assert csvOut.exists() : "CSV File not found when writing to subsystem " + subsystemname;
    }

    
}
