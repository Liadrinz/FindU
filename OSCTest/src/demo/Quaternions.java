package demo;

public class Quaternions
{
    public Quaternions(){}
    public double[] quaternConj(double[] q)
    {
        double[] qConj  = new double[q.length];
            qConj[0] = q[0];
            qConj[1] = -q[1];
            qConj[2] = -q[2];
            qConj[3] = -q[3];
        return qConj;
    }
    public double[] quaternProd(double[]a,double[] b)
    {
        double[] qProd = new double[a.length];
            qProd[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
            qProd[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
            qProd[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
            qProd[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
        return  qProd;
    }
    public double[][] quaternRotate(double[][] v,double[][] q)
    {
        int row = v.length;
        int col = v[0].length;
        double[][] temp = new double[row][col+1];
        for (int i = 0;i<row;i++)
        {
            temp[i][0] = 0;
            for (int j = 1;j<=col;j++)
            {
                temp[i][j] = v[i][j-1];
            }
        }
        double[][] v0XYZ =new double[q.length][4];
        for (int i = 0;i<q.length;i++)
        {
            v0XYZ[i] = quaternProd(quaternProd(q[i],temp[i]),quaternConj(q[i]));
        }
        for (int i = 0;i<row;i++)
        {
           for (int j = 1;j<4;j++)
           {
               v[i][j-1]  = v0XYZ[i][j];
           }
        }
        return v;
    }
}
