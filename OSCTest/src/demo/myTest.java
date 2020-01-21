package demo;

import java.io.IOException;
import java.lang.reflect.Array;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Collection;
import java.util.List;
import java.util.TimeZone;

import javax.xml.crypto.Data;


public class myTest 
{
	//public static OSCPortIn receiver = null;
	public static String getTime(){
		final Calendar c = Calendar.getInstance();
		c.setTimeZone(TimeZone.getTimeZone("GMT+8:00"));
		String mHour = String.valueOf(c.get(Calendar.HOUR_OF_DAY));//时
		String mMinute = String.valueOf(c.get(Calendar.MINUTE));//分
		String mSecond = String.valueOf(c.get(Calendar.SECOND));//秒
		String mMill = String.valueOf(c.get(Calendar.MILLISECOND));
		return ""+mHour+":"+mMinute+":"+mSecond+":"+mMill;
	}
	public static void main(String[] args) 
	{
		ArrayList<ArrayList<Double>> aList = new ArrayList<ArrayList<Double>>();
		int size = aList.size();
		IMUReader myIMUReader = new IMUReader();
		double[] myGyrX = myIMUReader.getGyrX(); 
		double[] myGyrY = myIMUReader.getGyrY(); 
		double[] myGyrZ = myIMUReader.getGyrZ(); 
		double[] myAccX = myIMUReader.getAccX();
		double[] myAccY = myIMUReader.getAccY();
		double[] myAccZ = myIMUReader.getAccZ();
		double[] myTime = myIMUReader.getTime();
//		Zero_Vec_Detector zVec_Detector = new Zero_Vec_Detector(myGyrX, myGyrY, myGyrZ, myAccX, myAccY, myAccZ);
//		int[] zupt = zVec_Detector.getZUPT();
//		double[] T = zVec_Detector.getT();
//		ZUPTaidedINS zuptAidedINS = new ZUPTaidedINS(myGyrX, myGyrY,myGyrZ, myAccX, myAccY, myAccZ, zupt);
//		double[][] x_h = zuptAidedINS.getX_h();
//		@SuppressWarnings("unused")
//		double[][] cov = zuptAidedINS.getCov();
//		System.out.println("v");
		double[] tempTime = new double[myAccX.length-1];
		double period = 1.0/256;
		int packnum = 256;
		PDRAlgorithms myPDRAlgorithms = new PDRAlgorithms();
		
		
		double[] initGyrX = new double[513];
		double[] initGyrY = new double[513];
		double[] initGyrZ = new double[513];
		double[] initAccX = new double[513];
		double[] initAccY = new double[513];
		double[] initAccZ = new double[513];
		for(int i=0;i<513;i++) 
		{
			initGyrX[i] = myGyrX[i];
			initGyrY[i] = myGyrY[i];
			initGyrZ[i] = myGyrZ[i];
			initAccX[i] = myAccX[i];
			initAccY[i] = myAccY[i];
			initAccZ[i] = myAccZ[i];
		}
		myPDRAlgorithms.initPDR(initGyrX, initGyrY, initGyrZ, initAccX, initAccY, initAccZ);
		int pack = 0;
		while (true) {
			double[] testGyrX = new double[packnum];
			double[] testGyrY = new double[packnum];
			double[] testGyrZ = new double[packnum];
			double[] testAccX = new double[packnum];
			double[] testAccY = new double[packnum];
			double[] testAccZ = new double[packnum];
			for(int i=0;i<packnum;i++) 
			{
				testGyrX[i] = myGyrX[i+513+packnum*pack];
				testGyrY[i] = myGyrY[i+513+packnum*pack];
				testGyrZ[i] = myGyrZ[i+513+packnum*pack];
				testAccX[i] = myAccX[i+513+packnum*pack];
				testAccY[i] = myAccY[i+513+packnum*pack];
				testAccZ[i] = myAccZ[i+513+packnum*pack];
			}
//			Zero_Vec_Detector zVec_Detector = new Zero_Vec_Detector(testGyrX, testGyrY, testGyrZ, testAccX, testAccY, testAccZ);
//			int[] zupt = zVec_Detector.getZUPT();
//			double[] T = zVec_Detector.getT();
//			ZUPTaidedINS zuptAidedINS = new ZUPTaidedINS(testGyrX, testGyrY, testGyrZ, testAccX, testAccY, testAccZ, zupt);
//			double[][] x_h = zuptAidedINS.getX_h();
//			@SuppressWarnings("unused")
//			double[][] cov = zuptAidedINS.getCov();
//			System.out.println("v");
			//if (pack%256==0) System.out.println("st: "+getTime());
			myPDRAlgorithms.updatePDR(null, testGyrX, testGyrY, testGyrZ, testAccX, testAccY, testAccZ, null, null, null, null, null);
			//if (pack%256==0) System.out.println("ed: "+getTime());
			pack++;
			//if (pack%256==0) {
				System.out.println(""+myPDRAlgorithms.getStepNumber());
				double stlen = myPDRAlgorithms.getLatestStepLength();
				System.out.println(""+stlen);
				System.out.print("");
			//}
		}
		
//		Thread thread = new Thread() {
//			@Override
//			public void run() {
//				try {
//					SocketAddress socketAddress = new InetSocketAddress("192.168.1.2",8001) ;
//					receiver = new OSCPortIn(socketAddress);
//					//SocketAddress socketAddress1 =receiver.getRemoteAddress();
//					//EchoOSCMessageListener echoOSCMessageListener = new EchoOSCMessageListener();
//					MessageSelector messageSelector = new OSCPatternAddressMessageSelector("");
//					OSCMessageListener messageListener = new OSCMessageListener() {
//						
//						@Override
//						public void acceptMessage(OSCMessageEvent arg0) {
//							// TODO Auto-generated method stub
//							System.out.println("message recieved");
//						}
//					};
//					OSCPacketListener listener = new OSCPacketListener() {
//						
//						@Override
//						public void handlePacket(OSCPacketEvent arg0) {
//							// TODO Auto-generated method stub
//							System.out.println("recieved");
//							receiver.getDispatcher().dispatchPacket(arg0);
//							//System.out.println(receiver.getDispatcher());
//							System.out.println(arg0.getSource().toString());
//							System.out.println(arg0.getPacket().toString());
//						}
//						
//						@Override
//						public void handleBadData(OSCBadDataEvent arg0) {
//							// TODO Auto-generated method stub
//							
//						}
//					};
//					receiver.getDispatcher().addListener(messageSelector, messageListener);
//					receiver.addPacketListener(listener);
//					receiver.startListening();
//					if (receiver.isListening())
//						System.out.println("Server is listening");
//					receiver.run();
//				} catch (Exception e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//					System.out.println("error " + e);
//				}
//			}
//		};
// 
//		thread.start();
		
		//myPDRAlgorithms.deg2red(0);
		//System.out.print("a");
	}
	
}
