package demo;

import java.io.*;
import java.util.ArrayList;
import java.util.List;

public class IMUHelper
{
    public IMUHelper(){
    	super();   
    }
    public final static String FileQuaternion = "data/LoggedData0307_Quaternion.csv";
//    public final static String FileAccAndGyr = "data/LoggedData0307_CalInertialAndMag.csv";
//    public final static String FileDataTime = "data/LoggedData0307_DateTime.csv";
//    public final static String FileAccAndGyr = "data/xIMU_Raw_CalInertialAndMag.csv";
//    public final static String FileDataTime = "data/TestDataTime.csv";
    public final static String FileAccAndGyr = "data/NGIMUearth_CalInertialAndMag.csv";
    public final static String FileDataTime = "data/TestDataTime.csv";
    public static  ArrayList<ArrayList<Double>> read(String FileType) throws IOException {
        FileReader myFile  = null;
        ArrayList<String> strList = new ArrayList<String>();
        ArrayList<ArrayList<Double>> doubList = new ArrayList<ArrayList<Double>>();
        try {
            myFile = new FileReader(FileType);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        BufferedReader  br = new BufferedReader(myFile);
        try {
        		String strName = br.readLine();
            String str = br.readLine();
            while (str!=null)
            {
            		ArrayList<Double> temp = new ArrayList<Double>();
            		String[] myValue = str.split(",");
            		for(int j = 0; j <myValue.length;j++) 
            		{
            			double k = Double.parseDouble(myValue[j]);
            			temp.add(k);
            		}
            		doubList.add(temp);
                str = br.readLine();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        return doubList;
    }
    public  ArrayList<ArrayList<Double>> readQuaternion() 
    {
    		ArrayList<ArrayList<Double>> doubList = new ArrayList<ArrayList<Double>>();
    		try {
    			doubList = read(FileQuaternion);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
    		return doubList;
    }
    public ArrayList<ArrayList<Double>> readAccAndGyr() 
    {
    		ArrayList<ArrayList<Double>> doubList = new ArrayList<ArrayList<Double>>();
    		try {
    			doubList = read(FileAccAndGyr);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
    		return doubList;
    }
    public ArrayList<ArrayList<Double>> readDataTime() 
    {
    		ArrayList<ArrayList<Double>> doubList = new ArrayList<ArrayList<Double>>();
    		try {
    			doubList = read(FileDataTime);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
    		return doubList;
    }
}
