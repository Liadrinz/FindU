package demo;

import java.util.ArrayList;

public class Algo {
	public static double deg2red(double a)
    {
        return a*Math.PI/180;
    }
	public int[] sign(double[] a , int offset)
    {
        int[] mResult = new int[a.length];
        for (int i = 0;i<a.length;i++)
        {
            if (a[i]>0) mResult[i] = 1+offset;
            else if (a[i]==0) mResult[i] = 0+offset;
            else mResult[i] = -1+offset;
        }
        return mResult;
    }
	public static int[] find(int[] a,int num)
    {
        // int[] mResult = new int[num];
        ArrayList<Integer> mResult = new ArrayList<Integer>();
        int k = 0;
        for (int i=0;i<a.length;i++)
        {
            if (a[i]!=0)
            {
                mResult.add(i);
                k++;
                if (k>=num)
                {
                    break;
                }
            }
        }
        int[] sum = new int[mResult.size()];
        for(int i = 0;i<mResult.size();i++)
        {
            sum[i] = mResult.get(i);
        }
        return sum;
    }
	public static int[] diff(int[] a)
    {
        int[] mResult=new int[a.length+1];
        for (int i = 0;i<a.length-1;i++)
        {
            mResult[i] = a[i+1] - a[i];
        }
        return mResult;
    }
	public static double mean(double[] a,int[] b)
    {
        double sum = 0;
        for (int i = 0; i<b.length;i++)
        {
            sum += a[b[i]];
        }
        return  sum/b.length;
    }
	public static double max_a (double[] T) 
	{
		double res = 0;
		for (int i = 0; i < T.length; i++) {
			if (res<T[i]) {
				res = T[i];
			}
		}
		return res;
	}
	public static double norm(double[] a)
    {
        double sum = 0;
        for (int i = 0;i<a.length;i++)
        {
            sum += a[i]*a[i];
        }
        sum = Math.sqrt(sum);
        return sum;
    }
}
