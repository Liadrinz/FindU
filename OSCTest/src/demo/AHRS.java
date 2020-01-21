package demo;

public class AHRS
{
    public double SamplePeriod = 1.0/256;
    public double[] Quaternion = {1,0,0,0};
    double kp = 2;
    double ki = 0;
    double kpInit =200;
    double initPeriod = 5;
    private double[] q = {1,0,0,0};
    private double[] intError = {0,0,0};                // intError is a  transposition vector
    private double KpRamped ;	
    public AHRS(double SamplePeriod,int kp,int kpInit)
    {
        this.SamplePeriod = SamplePeriod;
        this.kp = kp;
        this.kpInit = kpInit;

    }

    double[] cross(double[]a,double[] b)
    {
        double[] res = new double[a.length];
        res[0] = a[1]*b[2]-b[1]*a[2];
        res[1] = a[2]*b[0]-b[2]*a[0];
        res[2] = a[0]*b[1]-b[0]*a[1];
        return res;
    }
    public void updateIMU(double[] Gyroscope,double[] Accelerometer)
    {
    		Algo algo = new Algo();
        Quaternions qt = new Quaternions();
        double accelerometer_Norm = 0;
        for (int i = 0;i<Accelerometer.length;i++)                 //
        {
            accelerometer_Norm += Accelerometer[i]*Accelerometer[i];
        }
        accelerometer_Norm = Math.sqrt(accelerometer_Norm);
        if (accelerometer_Norm ==0)
        {
            System.out.println("warning from AHRS : Accelerometer magnitude is zero.");
            return;
        }
        else {
            for (int i = 0;i<Accelerometer.length;i++)
            {
                Accelerometer[i] =  Accelerometer[i]/accelerometer_Norm;
            }
        }
        //------------Compute error between estimated and measured direction of gravity------------------------------
        double[] v =new double[3] ;                                       // estimate gravity direction
        v[0] =2*(this.q[1]*this.q[3]-this.q[0]*this.q[2]);
        v[1] =2*(this.q[0]*this.q[1]+this.q[2]*this.q[3]);
        v[2] =Math.pow(this.q[0],2)-Math.pow(this.q[1],2)-Math.pow(this.q[2],2)+Math.pow(this.q[3],2);
        double[] error =  cross(v,Accelerometer);
        //------------------------------------------------------------------------------------------------------------
        //--------------------Compute ramped Kp value used during init period----------------------------------------
        //if (this.KpRamped>this.kp)
        //{
        //    intError[0] = 0;
        //    intError[1] = 0;
        //    intError[2] = 0;
        //    this.KpRamped = this.KpRamped - (this.kpInit - this.kp) / (this.initPeriod / this.SamplePeriod);
        //}
        //else
        //{
        //    this.KpRamped = this.kp;
            intError[0] += error[0];
            intError[1] += error[1];
            intError[2] += error[2];
        //}
        double[] Ref = new double[3];
        for (int i = 0;i<3;i++)
        {
            Ref[i] = Gyroscope[i] - this.kp*error[i] - this.ki*intError[i];
        }
        double[] pTemp = {0,Ref[0],Ref[1],Ref[2]};
        double[] pDot=qt.quaternProd(q,pTemp);

        for (int i=0;i<4;i++)
        {
            pDot[i] = 0.5*pDot[i];
            this.q[i] = this.q[i] + this.SamplePeriod * pDot[i];
        }
        for(int i =0 ;i<4;i++) 
        {
            this.q[i] = this.q[i]/algo.norm(q);
        }
        this.Quaternion = qt.quaternConj(q);
    }
    public void setQuaternion(double[] value)
    {
    		Algo algo = new Algo();
        Quaternions qt = new Quaternions();
        for (int i = 0;i<value.length;i++)
        {
            if (value[i]!=0) break;
            if (i==3&&value[i]==0)
            {
            	 	System.out.println("warning from AHRS : Accelerometer magnitude is zero.");
                return;
            }
        }
        for (int i = 0;i<value.length;i++)
        {
            value[i] = value[i]/algo.norm(value);
        }
            this.Quaternion = value;
        this.q = qt.quaternConj(value);
    }
}
