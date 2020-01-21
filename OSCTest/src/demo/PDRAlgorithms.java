package demo;
import java.text.AttributedCharacterIterator;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.TimeZone;

import demo.Algo;
import javax.swing.JFrame;

//import org.jfree.*;
//import org.jfree.data.category.DefaultCategoryDataset;

import biz.source_code.dsp.filter.FilterPassType;
import biz.source_code.dsp.filter.IirFilterCoefficients;
import biz.source_code.dsp.filter.IirFilterDesignExstrom;

public class PDRAlgorithms
{
    public static final String TAG = "PDRAlgorithms";
    private double[][] mQuaternion ;
    private double[] mQuat;

    private double[] mMagnetometerX;
    private double[] mMagnetometerY;
    private double[] mMagnetometerZ;
    private double[] mBattery;

    private int[] stationaryStart;
    private int[] stationaryEnd;
    private int stepNum=0;
    private int LastStepNum = 0;
    private double[][] pos;
    private double[][] stepLength;
    /*------------------       Array for stepLength                           ------------------*/
    private ArrayList<Double> accX = new ArrayList<Double>();
    private ArrayList<Double> accY = new ArrayList<Double>();
    private ArrayList<Double> accZ = new ArrayList<Double>();
    private ArrayList<Double> gyrX = new ArrayList<Double>();
    private ArrayList<Double> gyrY = new ArrayList<Double>();
    private ArrayList<Double> gyrZ = new ArrayList<Double>();
    private double[] mGyroscopeX;
    private double[] mGyroscopeY;
    private double[] mGyroscopeZ;
    private double[] mAcceleroemterX;
    private double[] mAcceleroemterY;
    private double[] mAcceleroemterZ;
    private int[] indexSel;
   // private StepNumChangeListener mStepNumChangeListener = null;
    private  AHRS AHRSalgorithm;

    public PDRAlgorithms() {
        // TODO Auto-generated constructor stub
        //this.mStepNumChangeListener = stepNumChangeListener;
    }
    public void initPDR(double[] GyroscopeX,double[] GyroscopeY,double[] GyroscopeZ,
                        double[] AcceleroemterX,double[] AcceleroemterY,double[] AcceleroemterZ)
    {

        for (int i = 0; i < GyroscopeX.length; i++) {
            gyrX.add(GyroscopeX[i]);
            gyrY.add(GyroscopeY[i]);
            gyrZ.add(GyroscopeZ[i]);
            accX.add(AcceleroemterX[i]);
            accY.add(AcceleroemterY[i]);
            accZ.add(AcceleroemterZ[i]);
//    			initGyrX[i] = myGyrX[i];
//    			initGyrY[i] = myGyrY[i];
//    			initGyrZ[i] = myGyrZ[i];
//    			initAccX[i] = myAccX[i];
//    			initAccY[i] = myAccY[i];
//    			initAccZ[i] = myAccZ[i];
        }
    }
    public void updatePDR(int[] Quaternion,double[] GyroscopeX,double[] GyroscopeY,double[] GyroscopeZ,
                          double[] AcceleroemterX,double[] AcceleroemterY,double[] AcceleroemterZ,
                          double[] MagnetometerX,double[] MagnetometerY,double[] MagnetometerZ,double[] Battery,double[] Time
    )
    {
        //System.out.println("updatePDR: add data st time: "+getTime());
        for (int i = 0;i<GyroscopeX.length;i++)
        {
            accX.add(AcceleroemterX[i]);
            accY.add(AcceleroemterY[i]);
            accZ.add(AcceleroemterZ[i]);
            gyrX.add(GyroscopeX[i]);
            gyrY.add(GyroscopeY[i]);
            gyrZ.add(GyroscopeZ[i]);
        }
        this.indexSel = new int[accX.size()];
        this.mQuaternion = new double[indexSel.length][4];
        this.mGyroscopeX = new double[indexSel.length];
        this.mGyroscopeY = new double[indexSel.length];
        this.mGyroscopeZ = new double[indexSel.length];
        this.mAcceleroemterX = new double[indexSel.length];
        this.mAcceleroemterY = new double[indexSel.length];
        this.mAcceleroemterZ = new double[indexSel.length];
//        Log.d(LOG_TAG,"indexSel: "+indexSel.length);
//        this.mMagnetometerX = MagnetometerX;
//        this.mMagnetometerY = MagnetometerY;
//        this.mMagnetometerZ = MagnetometerZ;
//        this.mBattery = Battery;
        for (int i = 0;i<indexSel.length;i++)
        {
            indexSel[i] = i;
        }
        for (int i = 0; i<this.indexSel.length; i++)
        {
            //mTime[i] = Time[this.indexSel[i]-1];
            mGyroscopeX[i] = gyrX.get(i);
            mGyroscopeY[i] = gyrY.get(i);
            mGyroscopeZ[i] = gyrZ.get(i);
            mAcceleroemterX[i] = accX.get(i);
            mAcceleroemterY[i] = accY.get(i);
            mAcceleroemterZ[i] = accZ.get(i);
        }
        /*----------------------------------------------------------------------------------*/
        int[] stationary =new int[this.indexSel.length];
        double[] acc_mag =new double[this.indexSel.length];
        for (int i = 0;i<this.indexSel.length;i++)
        {
            //double temp= Math.sqrt(mAcceleroemterX[i]*mAcceleroemterX[i]+mAcceleroemterY[i]*mAcceleroemterY[i]+mAcceleroemterZ[i]*mAcceleroemterZ[i])-0.98;
            acc_mag[i] = Math.abs(Math.sqrt(mAcceleroemterX[i]*mAcceleroemterX[i]+mAcceleroemterY[i]*mAcceleroemterY[i]+mAcceleroemterZ[i]*mAcceleroemterZ[i])-0.98) ;
        }

        double samplePeriod = 1.0/256;
        //System.out.println( "updatePDR: add data ed time: "+getTime());
        //System.out.println("updatePDR: filter st time: "+getTime());
        //-------------------------------------------------------------------------------------------------------
        //------------------------bandpass fliter-----------------------------------------------------------------
        IirFilterCoefficients iirFilterCoefficients3;
        double low = 3.6;
        double high = 0.0001;
        iirFilterCoefficients3 = IirFilterDesignExstrom.design(FilterPassType.bandpass, 2, high/(1/samplePeriod), low/(1/samplePeriod));
        double[] acc_magFilt1 = filter( iirFilterCoefficients3.b, iirFilterCoefficients3.a,acc_mag);
        //-------------------------stationary--------------------------------------------------------------------
        for (int i=0;i<acc_magFilt1.length;i++)
        {
            if (acc_magFilt1[i]<0.05) stationary[i] = 1;
            else stationary[i] = 0;
            // Log.d(LOG_TAG,"stationary "+i+": "+stationary[i]);
        }
        int[] stStartTemp = new int[stationary.length+1];
        int[] stEndTemp = new int[stationary.length+1];
        int[] tempDiff = Algo.diff(stationary);
        for (int i = 0;i<stationary.length;i++)
        {
            stStartTemp[i+1] = tempDiff[i];
            stEndTemp[i+1] = tempDiff[i];
        }
        for (int i = 0; i<stStartTemp.length-1;i++)
        {
            if (stStartTemp[i] == -1)  stStartTemp[i] = 1;
            else stStartTemp[i] = 0;
            if (stEndTemp[i]==1)  stEndTemp[i] = 1;
            else stEndTemp[i] = 0;
        }
        stStartTemp[stStartTemp.length-1] = stStartTemp[stStartTemp.length-2];
        stEndTemp[stStartTemp.length-1] = stEndTemp[stStartTemp.length-2];
        stationaryStart = Algo.find(stStartTemp,stStartTemp.length);
        stationaryEnd = Algo.find(stEndTemp,stEndTemp.length);
        double[] acc_mag2 = null;
        if (stationaryEnd.length==0)
        {
            acc_mag2 = new double[1+30];
        }else
        {
            if ((stationaryEnd[stationaryEnd.length-1]+30)<stationary.length) {
                int i;
                for (i = 0; i < 30; i++) {
                    if (stationary[stationaryEnd[stationaryEnd.length-1]+i]==0) {
                        if (stationaryEnd.length==1) {
                            acc_mag2 = new double[1+30];
                            break;
                        }
                        else {
                            acc_mag2 = new double[stationaryEnd[stationaryEnd.length-2]+30];
                            break;
                        }
                    }
                    acc_mag2 = new double[stationaryEnd[stationaryEnd.length-1]+30];
                }

            }else {
                int i;
                for (i = 0; i < stationary.length-(stationaryEnd[stationaryEnd.length-1]); i++) {
                    if (stationary[stationaryEnd[stationaryEnd.length-1]+i]==0) {
                        if (stationaryEnd.length==1) {
                            acc_mag2 = new double[1+30];
                            break;
                        }
                        else {
                            acc_mag2 = new double[stationaryEnd[stationaryEnd.length-2]+stationary.length-stationaryEnd[stationaryEnd.length-1]];
                            break;
                        }
                    }
                    acc_mag2 = new double[stationaryEnd[stationaryEnd.length-1]+stationary.length-stationaryEnd[stationaryEnd.length-1]];
                }

            }
        }
        for (int i = 0;i<acc_mag2.length-30;i++)
        {
            acc_mag2[i] = acc_mag[i];
        }
        for (int i = acc_mag2.length-30; i < acc_mag2.length; i++) {
            acc_mag2[i] = 0.00001;
        }
        if (stationaryEnd.length>0)
        {
            //------------------------highpass fliter-----------------------------------------------------------------
            IirFilterCoefficients iirFilterCoefficients1 ;
            double flitCutOff = 0.001;
            iirFilterCoefficients1 = IirFilterDesignExstrom.design(FilterPassType.highpass,1, (flitCutOff)/(1/samplePeriod), (flitCutOff)/(1/samplePeriod));
            acc_mag2 = myFiltfilt(acc_mag2,iirFilterCoefficients1.a,iirFilterCoefficients1.b);
            for (int i=0;i<acc_mag2.length;i++)
            {
                acc_mag2[i] = Math.abs(acc_mag2[i]);
            }
//        //--------------------------------------------------------------------------------------------------------
//        //------------------------lowpass fliter-----------------------------------------------------------------
            IirFilterCoefficients iirFilterCoefficients2;
            flitCutOff = 5;
            iirFilterCoefficients2 = IirFilterDesignExstrom.design(FilterPassType.lowpass,1,flitCutOff/(1/samplePeriod),flitCutOff/(1/samplePeriod));
            acc_mag2 = myFiltfilt(acc_mag2,iirFilterCoefficients2.a,iirFilterCoefficients2.b);
            for (int i=0;i<acc_mag2.length;i++)
            {
                acc_mag2[i] = Math.abs(acc_mag2[i]);
            }
            //-------------------------------------------------------------------------------------------------------
            //-------------------------stationary--------------------------------------------------------------------
            stationary = new int[acc_mag2.length];
            for (int i=0;i<acc_mag2.length;i++)
            {
                if (acc_mag2[i]<0.05) stationary[i] = 1;
                else stationary[i] = 0;
                // Log.d(LOG_TAG,"stationary "+i+": "+stationary[i]);
            }
            stStartTemp = new int[stationary.length+1];
            stEndTemp = new int[stationary.length+1];
            tempDiff = Algo.diff(stationary);
            for (int i = 0;i<stationary.length;i++)
            {
                stStartTemp[i+1] = tempDiff[i];
                stEndTemp[i+1] = tempDiff[i];
            }
            for (int i = 0; i<stStartTemp.length-1;i++)
            {
                if (stStartTemp[i] == -1)  stStartTemp[i] = 1;
                else stStartTemp[i] = 0;
                if (stEndTemp[i]==1)  stEndTemp[i] = 1;
                else stEndTemp[i] = 0;
            }
//            for (int i = 0; i<stStartTemp.length-1;i++)
//            {
//                if (stStartTemp[i+1]-stStartTemp[i] == -1)  stStartTemp[i] = 1;
//                else stStartTemp[i] = 0;
//                if (stEndTemp[i+1]-stEndTemp[i] == 1)  stEndTemp[i] = 1;
//                else stEndTemp[i] = 0;
//            }
//            stStartTemp[stStartTemp.length-1] = stStartTemp[stStartTemp.length-2];
//            stEndTemp[stStartTemp.length-1] = stEndTemp[stStartTemp.length-2];
            stationaryStart = Algo.find(stStartTemp,stStartTemp.length);
            stationaryEnd = Algo.find(stEndTemp,stEndTemp.length);
            stepNum = stationaryEnd.length;
//            Log.d(TAG, "updatePDR: filter ed time: "+getTime());
 //           Log.d(TAG, "updatePDR: computer data st time: "+getTime());
            //--------------------------Compute orientation----------------------------------------------------------
            double[][] quat = new double[stationary.length][4];
//            for (int j = 0; j< stationary.length; j++)
//            {
//                for (int i = 0;i<4;i++)
//                {
//                    quat[j][i] = 0;
//                }
//            }
//        for (int j = mQuat.get(0).size(); j<indexSel.length; j++)
//        {
//            for (int i = 0;i<4;i++)
//            {
//                quat[j][i] = 0;
//            }
//        }

//        int initPeriod = 2;
//        double[] mTime2 = new double[indexSel.length];
//        for (int i = 0; i<indexSel.length;i++)
//        {
//            mTime2[i] = mTime[indexSel[i]] - mTime[indexSel[0]] - initPeriod;
//        }
            AHRSalgorithm = new AHRS(1.0/256,1,1);
            int[] indexSel2 = new int[513];
            for (int i = 0; i<indexSel2.length;i++)
            {
                indexSel2[i] = i;
            }
            for (int i = 0;i<2000;i++)
            {
                double[] gy = {0,0,0};
                double[] meanAcc = new double[3];
                meanAcc[0] = Algo.mean(mAcceleroemterX,indexSel2);
                meanAcc[1] = Algo.mean(mAcceleroemterY,indexSel2);
                meanAcc[2] = Algo.mean(mAcceleroemterZ,indexSel2);
                AHRSalgorithm.updateIMU(gy,meanAcc);
            }
            for (int i = 0 ;i< stationary.length;i++)
            {
                double[] degtoredgy =new  double[3] ;
                double[] tAcc =new  double[3];
                if (stationary[i]==1)
                {
                    AHRSalgorithm.kp = 0.5;
                }
                else
                {
                    AHRSalgorithm.kp = 0;
                }
                degtoredgy[0] = Algo.deg2red(mGyroscopeX[i]);
                degtoredgy[1] = Algo.deg2red(mGyroscopeY[i]);
                degtoredgy[2] = Algo.deg2red(mGyroscopeZ[i]);
                tAcc[0] = mAcceleroemterX[i];
                tAcc[1] = mAcceleroemterY[i];
                tAcc[2] = mAcceleroemterZ[i];
                AHRSalgorithm.updateIMU(degtoredgy,tAcc);
                quat[i] = AHRSalgorithm.Quaternion;
            }
            mQuat = quat[quat.length-1];
            //-------------------------------------------------------------------------------------------------------
            //--------------------------Compute translation Acc------------------------------------------------------
            Quaternions qt = new Quaternions();
            double[][] temp1 =new double[stationary.length][3];
            //double[][] acc = new double[mAcceleroemterX.length][3];
            for (int i = 0;i<stationary.length;i++)
            {
                temp1[i][0] = mAcceleroemterX[i];
                temp1[i][1] = mAcceleroemterY[i];
                temp1[i][2] = mAcceleroemterZ[i];
            }
            double[][] temp2 =new double[quat.length][4];
            for (int i = 0;i<quat.length;i++)
            {
                temp2[i] = qt.quaternConj(quat[i]);
            }
            double[][] acc = qt.quaternRotate(temp1,temp2);
            for (int i = 0;i<acc.length;i++)
            {
                for (int j = 0;j<acc[0].length;j++)
                {
                    acc[i][j] = acc[i][j] *9.81;
                    if (j==2) acc[i][j] = acc[i][j]-9.81;
                }
            }
//            for (int i = 0;i<acc.length;i++)
//            {
//                acc[i][2] = acc[i][2] - 9.81;
//            }
            //-------------------------------------------------------------------------------------------------------
            //--------------------------Compute translation vel------------------------------------------------------
            double[][] vel  =new double[stationary.length][3];;
//            for (int i = 0; i< stationary.length; i++)
//            {
//                for (int j = 0;j<3;j++)
//                {
//                    vel[i][j] = 0;
//                }
//            }
            for (int i = 1;i<vel.length;i++)
            {
                for (int j = 0;j<vel[i].length;j++)
                {
                    if (stationary[i]!=1) vel[i][j] = vel[i-1][j]+ acc[i][j]*samplePeriod;
                }
                if (stationary[i]==1)
                {
                    vel[i][0] =0;
                    vel[i][1] =0;
                    vel[i][2] =0;
                }
            }
            //-------------------------------------------------------------------------------------------------------
            //--------------------------Compute integral drift during non-stationary periods-------------------------
            double[][] veldrift=new double[vel.length][3];
//            for (int i = 1;i<vel.length;i++)
//            {
//                for (int j = 0;j<vel[i].length;j++)
//                {
//                    veldrift[i][j] = 0;
//                }
//            }

            for (int i = 0;i<stationaryEnd.length;i++)
            {
                //Log.d(LOG_TAG, "st"+i+": "+stationaryStart[i]);
                //Log.d(LOG_TAG, "ed"+i+": "+stationaryEnd[i]);
                double[] driftRate =new double[3];
                driftRate[0]  = vel[stationaryEnd[i]-1][0]/(stationaryEnd[i]-stationaryStart[i]);
                driftRate[1]  = vel[stationaryEnd[i]-1][1]/(stationaryEnd[i]-stationaryStart[i]);
                driftRate[2]  = vel[stationaryEnd[i]-1][2]/(stationaryEnd[i]-stationaryStart[i]);
                if ((stationaryEnd[i]-stationaryStart[i])<0)
                {
                    return;
                }
//                int[] myEnum = new int[(stationaryEnd[i]-stationaryStart[i])];
//                for (int j = 0;j<(stationaryEnd[i]-stationaryStart[i]);j++)
//                {
//                    myEnum[j] = j;
//                }
               // double[][] drift = new double[myEnum.length][3];
                double[][] drift = new double[stationaryEnd[i]-stationaryStart[i]][3];
                for (int l = 0;l<(stationaryEnd[i]-stationaryStart[i]);l++)
                {
//                    drift[l][0] = myEnum[l]* driftRate[0];
//                    drift[l][1] = myEnum[l]* driftRate[1];
//                    drift[l][2] = myEnum[l]* driftRate[2];
                    drift[l][0] = l* driftRate[0];
                    drift[l][1] = l* driftRate[1];
                    drift[l][2] = l* driftRate[2];
                }
                for (int m = stationaryStart[i]-1;m<stationaryEnd[i]-2;m++)
                {
                    veldrift[m] =drift[m-stationaryStart[i]+1];
                }
            }
            for (int i = 0;i<vel.length;i++)
            {
                for (int j = 0;j<3;j++)
                {
                    vel[i][j] =vel[i][j] -veldrift[i][j];
                }
            }
            pos = new double[vel.length][3];
            int[] posX = new int[vel.length];
            int[] posY = new int[vel.length];
            int[] posZ = new int[vel.length];
            for(int i = 3;i<pos.length;i++)
            {
                pos[i][0] = pos[i-1][0]+vel[i][0]*samplePeriod;
                pos[i][1] = pos[i-1][1]+vel[i][1]*samplePeriod;
                pos[i][2] = pos[i-1][2]+vel[i][2]*samplePeriod;
                //posX[i] = (int)(pos[i][0]*50);
                //posY[i] = (int)(pos[i][1]*200);
                //posZ[i] = (int)(pos[i][2]*50);
                //Log.d(LOG_TAG, "posx: "+pos[i][0]);
            }
            if (stationaryEnd.length>0){
                stepLength = new double[stationaryEnd.length][3];
                for(int i = 0;i<stationaryEnd.length;i++)
                {
                    stepLength[i][0] = pos[stationaryEnd[i]][0] - pos[stationaryStart[i]][0];
                    stepLength[i][1] = pos[stationaryEnd[i]][1] - pos[stationaryStart[i]][1];
                    stepLength[i][2] = pos[stationaryEnd[i]][2] - pos[stationaryStart[i]][2];
                }
            }
        }
//        Log.d(TAG, "updatePDR: computer data ed time: "+getTime());
    }

    public int getStepNumber()
    {
        return stepNum;
    }
    public double getLatestStepLength()
    {
        if (stepLength==null) return 0;
        else {
            if (stepLength.length == 0)
            {
                return 0;
            }
            else 
            {
//            	if (isStepChange) {
//            		if (reslutStep!=null&&reslutStep.size()>2) {
//                		reslutStep.remove(0);
//    				}
//            		reslutStep.add(Math.sqrt(stepLength[stepLength.length-1][0]*stepLength[stepLength.length-1][0]+stepLength[stepLength.length-1][1]*stepLength[stepLength.length-1][1]+stepLength[stepLength.length-1][2]*stepLength[stepLength.length-1][2]));
//            		isStepChange =false;
//            	}
//            		double[]res =new double[reslutStep.size()];
//            	for (int i = 0; i < res.length; i++) 
//    				{
//            		res[i] = reslutStep.get(i);
//				}
//            	return res;
            	return Math.sqrt(stepLength[stepLength.length-1][0]*stepLength[stepLength.length-1][0]+stepLength[stepLength.length-1][1]*stepLength[stepLength.length-1][1]+stepLength[stepLength.length-1][2]*stepLength[stepLength.length-1][2]);
            }
        }
    }
    public double[] getLatestPos()
    {
        if (pos==null) return new double[]{0,0,0};
        else
        {
            if (pos.length==0) return new double[]{0,0,0};
            else return pos[pos.length-1];
        }

    }
    public double[] getLatestQuat()
    {
        return mQuat;
    }
    private static double[] filter(double[] b, double[] a, double[] x) {
        int nx = x.length;
        int na = a.length;
        int nb = b.length;

        double[] y = new double[nx];
        for (int k = 0; k < nx; k++) {
            y[k] = 0;
            for (int i = 0; i < nb; i++) {
                if (k - i >= 0 && k - i < nx) {
                    y[k] += b[i] * x[k - i];
                }
            }
            for (int i = 1; i < na; i++) {
                if (k - i >= 0 && k - i < nx) {
                    y[k] -= a[i] * y[k - i];
                }
            }
            if (Math.abs(a[0] - 1) > 1.e-9) {
                y[k] /= a[0];
            }

        }
        return y;
    }


    public synchronized double[] myFiltfilt(double[] signal, double[] a, double[] b)   //filtfilt
    {
        ArrayList<Double> tempS = new ArrayList<Double>();
        ArrayList<Double> tempA = new ArrayList<Double>();
        ArrayList<Double> tempB = new ArrayList<Double>();

        for (int i = 0; i < signal.length; i++) {
            tempS.add(signal[i]);
        }
        for (int i = 0; i < a.length; i++) {
            tempA.add(a[i]);
        }
        for (int i = 0; i < b.length; i++) {
            tempB.add(b[i]);
        }
        ArrayList<Double> tempResult = Filtfilt.doFiltfilt(tempB, tempA, tempS);
        int arraysize = tempResult.size();
        double[] myResult = new double[arraysize];
        for (int i = 0; i < arraysize; i++) {
            myResult[i] = tempResult.get(i);
        }
        return myResult;
    }
    private String getTime(){
        final Calendar c = Calendar.getInstance();
        c.setTimeZone(TimeZone.getTimeZone("GMT+8:00"));
        String mHour = String.valueOf(c.get(Calendar.HOUR_OF_DAY));//时
        String mMinute = String.valueOf(c.get(Calendar.MINUTE));//分
        String mSecond = String.valueOf(c.get(Calendar.SECOND));//秒
        String mMill =  String.valueOf(c.get(Calendar.MILLISECOND));
        return ""+mHour+":"+mMinute+":"+mSecond+":"+mMill;
    }
}