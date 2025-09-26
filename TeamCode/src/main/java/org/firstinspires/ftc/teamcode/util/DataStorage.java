package org.firstinspires.ftc.teamcode.util;

import android.os.Environment;

import java.io.*;

public class DataStorage {



    private static final String FILENAME = "heading.txt";

    public static void saveHeading(double heading) {
        try {
            File file = new File(Environment.getExternalStorageDirectory(), FILENAME);
            FileWriter writer = new FileWriter(file);
            writer.write(String.valueOf(heading));
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static double loadHeading() {
        try {
            File file = new File(Environment.getExternalStorageDirectory(), FILENAME);
            BufferedReader reader = new BufferedReader(new FileReader(file));
            return Double.parseDouble(reader.readLine());
        } catch (IOException | NumberFormatException e) {
            e.printStackTrace();
            return 0;
        }
    }

}
