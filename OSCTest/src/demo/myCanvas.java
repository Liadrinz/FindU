package demo;

import java.awt.Graphics;

import javax.swing.JFrame;
import javax.swing.JPanel;

public class myCanvas extends JPanel {
	public int[] X;
	public int[] Y;
	public myCanvas(int[]posx,int[] posy) 
	{
		// TODO Auto-generated constructor stub
		X = new int[posx.length];
		Y = new int[posy.length];
		X = posx;
		Y = posy;
	}
	@Override
	protected void paintComponent(Graphics arg0) {
		// TODO Auto-generated method stub
		super.paintComponent(arg0);
		System.out.print("enter pain");
		for(int i = 0;i<X.length-1;i++) 
		{
			arg0.drawLine(X[i], Y[i], X[i+1], Y[i+1]);
		}
	}
}
