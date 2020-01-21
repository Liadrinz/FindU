package demo;

import java.util.ArrayList;

public class IMUReader 
{
    private double[] mGyroscopeX;
    private double[] mGyroscopeY;
    private double[] mGyroscopeZ;
    private double[] mAcceleroemterX;
    private double[] mAcceleroemterY;
    private double[] mAcceleroemterZ;
    double[] myTime;
    public IMUReader() 
    {
    		//super();   
    		IMUHelper mt = new IMUHelper();
    		ArrayList<ArrayList<Double>> myAccAndGyr = mt.readAccAndGyr();
    		ArrayList<ArrayList<Double>> myDataTime = mt.readDataTime();
    		int arraySize1 = myAccAndGyr.size();
    		int arraySize2 = myDataTime.size();
    		mGyroscopeX = new double[arraySize1];
    		mGyroscopeY = new double[arraySize1];
    		mGyroscopeZ = new double[arraySize1];
    		mAcceleroemterX = new double[arraySize1];
    		mAcceleroemterY = new double[arraySize1];
    		mAcceleroemterZ = new double[arraySize1];
    		myTime = new double[arraySize2];
    		for(int i = 0;i< arraySize1 ;i++) 
    		{
    			//if kf algo gyr add *pi/180 acc add *9.81
    			mGyroscopeX[i] = myAccAndGyr.get(i).get(1);
    			mGyroscopeY[i] = myAccAndGyr.get(i).get(2);
    			mGyroscopeZ[i] = myAccAndGyr.get(i).get(3);
    			mAcceleroemterX[i] = myAccAndGyr.get(i).get(4);
    			mAcceleroemterY[i] = myAccAndGyr.get(i).get(5);
    			mAcceleroemterZ[i] = myAccAndGyr.get(i).get(6);
    		}
    		for(int i = 0;i< arraySize2 ;i++) 
    		{
    			myTime[i] = myDataTime.get(i).get(0);
    		}
    };
    public double[] getGyrX() 
	{
		return mGyroscopeX;
	}
    public double[] getGyrY() 
	{
		return mGyroscopeY;
	}
    public double[] getGyrZ() 
	{
		return mGyroscopeZ;
	}
    public double[] getAccX() 
	{
		return mAcceleroemterX;
	}
    public double[] getAccY() 
	{
		return mAcceleroemterY;
	}
    public double[] getAccZ() 
	{
		return mAcceleroemterZ;
	}
    public double[] getTime() 
	{
		return myTime;
	}
}
