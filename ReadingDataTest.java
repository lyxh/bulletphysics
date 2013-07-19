package com.bulletphysics;

import java.io.IOException;

public class ReadingDataTest {
	private static int totalDataNum=11989;
 	private static float[][] trackingData=new float[341][3];
	private static String caseDataPath="D:\\Yixin\\model\\Case_";
	private static String caseCountPath="D:\\Yixin\\model\\Case_Count_Model_2.txt";
	public static int[] caseCount=new int[27];

	public static void main(String [ ] args) throws IOException
	{
		caseCount=BasicDemo.readCaseCount(caseCountPath);
		int highest = caseCount[0]; // note: don't do this if the array could be empty
		for(int i = 1; i < caseCount.length; i++) {
		    if(highest<caseCount[i]) {highest = caseCount[i];}
		}
		totalDataNum=highest;
		trackingData=BasicDemo.readDistributionData(caseDataPath, 2);
	}
}
