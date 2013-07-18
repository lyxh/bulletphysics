package com.bulletphysics;

import static com.bulletphysics.demos.opengl.IGL.GL_LINES;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.math.BigDecimal;
import java.util.ArrayList;

import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;
import javax.vecmath.Tuple2d;
import javax.vecmath.Vector3f;
import javax.vecmath.Vector4f;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.InternalTickCallback;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.BasicDemo;

/**
 * The model that is similar to model 1. Only forward, side distance and angle rotated are taken into account
 * But the distribution is drawn from the original data set.
 * @author ssr
 *
 */
public class Model2 extends InternalTickCallback{	
	private static int range=20;//how far away the termite could sense
	private static float termiteHalfLen;
	private static int[] values=new int[4];
	private static double angleRange=Math.PI/2;
	private static float dishRadius=BasicDemo.getDishRadius();
	private static float small_dis=5;
	private static float large_dis=20;


 	private ArrayList<float[][]> trackingData=new ArrayList<float[][]>();
	private int[] caseCount=new int[27];
	public static int[] inputDistribution= new int[27];
	private int pullDownForce=5;
	public static int[] getInputDis(){return inputDistribution;}
	private static boolean applySame=false;
	private static int continuing=5;
	private String caseDataPath="D:\\Yixin\\model\\Case_Data_Model_2.txt";
	private String caseCountPath="D:\\Yixin\\model\\Case_Count_Model_2.txt";
	private DynamicsWorld dynamicsWorld;
	private IGL gl;
	
	public Model2(DynamicsWorld dy, IGL gl) {
		this.dynamicsWorld=dy;
		this.gl=gl;
		/*this.trackingData= readDistributionData(caseDataPath);
		this.caseCount=readCaseCount(caseCountPath);
		int highest = caseCount[0]; // note: don't do this if the array could be empty
		for(int i = 1; i < caseCount.length; i++) {
		    if(highest<caseCount[i]) highest = caseCount[i];
		}
		this.totalDataNum=highest;
		*/
		this.trackingData= BasicDemo.getData();
		this.caseCount=BasicDemo.getCaseCount();}
	

	public void internalTick(DynamicsWorld dynamicsWorld, float timeStep) {	
		ObjectArrayList<RigidBody> termites= BasicDemo.getTermites();
		ArrayList<ArrayList<Vector3f>> posList=BasicDemo.getPositionList();
		ArrayList<ArrayList<Quat4f>> oriList=BasicDemo.getOrientationList();
		 for (int j=0; j<termites.size(); j++) {
	    	RigidBody body= termites.get(j);	
			Vector3f position= new Vector3f(0,0,0);
			//get the position and orientation of each termite                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
			position=body.getCenterOfMassPosition(position);
			Quat4f orientation=new Quat4f();
			orientation=body.getOrientation(orientation);
			posList.get(j).add(position);
			oriList.get(j).add(orientation);
			
			//for each termite, classify the input condition
			float center_x=position.x;
			float center_y=position.y;
			float angle=getAngle(orientation);
			//position, angle correct 
			float terLen=BasicDemo.getTermiteLen();
			termiteHalfLen=(terLen/2);
			float head_x=(float) (center_x+termiteHalfLen*Math.cos(angle));
			float head_y=(float) (center_y+termiteHalfLen*Math.sin(angle));
			float tail_x=(float) (center_x-termiteHalfLen*Math.cos(angle));
			float tail_y=(float) (center_y-termiteHalfLen*Math.sin(angle));
			
			//draw the termites direction
			Vector3f from=new Vector3f(head_x,head_y,position.z);
			float front_to_x=(float) (head_x+range*Math.cos(angle));
			float front_to_y=(float) (head_y+range*Math.sin(angle));
			Vector3f front_to=new Vector3f(front_to_x,front_to_y,position.z);
			
			float left_to_x=(float) (head_x+range*Math.cos(angle+Math.PI/2));
			float left_to_y=(float) (head_y+range*Math.sin(angle+Math.PI/2));
			Vector3f left_to=new Vector3f(left_to_x,left_to_y,position.z);
			
			float right_to_x=(float) (head_x+range*Math.cos(angle-Math.PI/2));
			float right_to_y=(float) (head_y+range*Math.sin(angle-Math.PI/2));
			Vector3f right_to=new Vector3f(right_to_x,right_to_y,position.z);
			Vector3f color=new Vector3f(0,1,1);
			//draw the line of the forward,left,right direction
			drawLine(from,front_to,color);
			drawLine(from,left_to,color);
			drawLine(from,right_to,color);
			
			//1.check for wall.tested
			for (int dir=0;dir<4;dir++){
				values[dir]=0;
	             double point_x=head_x+range*Math.cos(angle-angleRange*dir);
	             double point_y=head_y+range*Math.sin(angle-angleRange*dir);
	             if ((point_x*point_x+point_y*point_y)>dishRadius*dishRadius){ values[dir]=1;}
			}	 
			
			//2.check for other termites
			for (int otherTer=0; otherTer<BasicDemo.getTermiteCount();otherTer++){
				if (j!=otherTer){
					//First, calculate the distance between two termites(center of the other termite- head position of this termite)
					Vector3f other_position= new Vector3f(0,0,0);
					RigidBody other_body=termites.get(otherTer);
					other_position= other_body.getCenterOfMassPosition(other_position);
					float other_x=other_position.x;
					float other_y=other_position.y;
					Quat4f other_orientation=new Quat4f();
					other_orientation= other_body.getOrientation(other_orientation);
					float other_angle=getAngle(other_orientation);
					float other_head_x=(float) (other_position.x+termiteHalfLen*Math.cos(other_angle)); //cos, sin takes in radians!
					float other_head_y=(float) (other_position.y+termiteHalfLen*Math.sin(other_angle));
					float other_tail_x=(float) (other_position.x-termiteHalfLen*Math.cos(other_angle));
					float other_tail_y=(float) (other_position.y-termiteHalfLen*Math.sin(other_angle));
					//check if the termite is too close
					if (Math.pow((other_x-head_x),2)+Math.pow((other_y-head_y),2)<range*range || Math.pow((other_head_x-head_x),2)+Math.pow((other_head_y-head_y),2)<range*range || Math.pow(other_tail_x-head_x,2)+Math.pow(other_tail_y-head_y,2)<range*range){
						//if the termite is close, check in which quadrant it is.
						float dis_angle=(float) Math.atan2(other_y-head_y,other_x-head_x);
						float angleChange=getAngleChange(angle, dis_angle);
						int direction=getDirectionFromAngle((double)angleChange);
						values[direction]=2;				
					}
				}
			}
			
			//Get the input case number
			int caseNum=values[0]+values[1]*3+values[3]*3*3;
			System.out.println(caseNum);
			inputDistribution[caseNum]+=1;
			int caseCount=this.caseCount[caseNum];
			
			//1.set velocity
			Vector3f localforce=getForceAngleFromDistribution(caseNum,caseCount);
			float rotatedAngle=localforce.z;
			Vector3f globalForce=getGlobalForce(localforce,body);
			globalForce.z=pullDownForce;
			body.setLinearVelocity(globalForce);
			
			//2.rotate the body
			//System.out.println(rotatedAngle);
			Transform tr=new Transform();
			tr=body.getCenterOfMassTransform(tr);
		    tr.basis.rotZ(rotatedAngle);
		    body.setCenterOfMassTransform(tr);
		    }
	}
	
	
	
	/**
	 * The angle is in decimal, original  -->0; left:-pi, right:+pi
	 * Want: <--0; left:-pi;right+pi
	 * Examples: 0->pi,-pi/4->pi*3/4,pi/4->-pi*3/4
	 * @param angle
	 * @return
	 */
	private float changeAngle(float angle) {
		//first, flip the sigh
		angle=angle*(float)-1;
		//complement it with pi
		if (angle>=0){angle=(float) (Math.PI-angle);}
		else{angle=(float) (-Math.PI-angle);}
		return angle;
	}

	/**
	 * 
	 * @param data
	 * @return
	 */
	 private Vector3f getForceAngleFromDistribution(int caseNum,int caseCount) {
        float x=10;
	    float y=0;
	    float angle=0;
		if (caseNum!=0){
		        int random=(int) (Math.random()*(float)caseCount);
		        if(random>=caseCount & (int)caseCount!=0){random=caseCount-1;}
		          x=this.trackingData.get(caseNum)[random][0];
		          y=this.trackingData.get(caseNum)[random][1];
		          angle=this.trackingData.get(caseNum)[random][2];
				
		 }
		 Vector3f force=new Vector3f(x,y,angle);
		return force;
	}


	

	
	private int[] readCaseCount(String filePath) {
		int[] result=new int[27];
		byte[] buffer = new byte[(int) new File(filePath).length()];
	    BufferedInputStream f = null;
	    try {f = new BufferedInputStream(new FileInputStream(filePath));
	        f.read(buffer);
	        if (f != null) try { f.close(); } catch (IOException ignored) { }} 
        catch (IOException ignored) { System.out.println("File not found or invalid path.");}			    
	    String[] strings=(new String(buffer)).split("\\s+");
	    for (int i=0; i<strings.length;i++){
	    	result[i]=Integer.valueOf(strings[i]);
	    }
		return result;
	}
	
     /**
      * front is 0, left is 1, back is 2, right is 3.
      * @param angle
      * @return
      */
	private int getDirectionFromAngle(double angle) {
		// TODO Auto-generated method stub
		 int result=0;
		 if (angle<Math.PI/4 && angle>-Math.PI/4){result=0;}
		 if (angle<Math.PI/4*3 && angle>Math.PI/4){result=3;}
		 if (angle<-Math.PI/4 && angle>-Math.PI/4*3){result=1;}
		 if (angle<-Math.PI/4*3 || angle>Math.PI/4*3){result=2;}
		 return result;
	}

	

	/**
	  * 
	  * @param orientation
	  * @return
	  */
	  private float getAngle(Quat4f orientation){
		    float w=orientation.w;
			float x=orientation.x;
			float y=orientation.y;
			float z=orientation.z;
			Vector3f euler=new Vector3f(0,0,0);
			euler.x=(float) Math.atan2(2.0 * (w*x + y*z),1-2*( y*y + x*x));
			euler.y=(float) Math.atan2(2.0 * (w*z + x*y),1-2*( y*y + z*z));
			euler.z=(float) Math.asin(2.0 * (w*y - x*z));
		    return euler.y;		  //we want yaw
	  }

	  
	  /**
	   * Calculate the angle difference between angle 2 and angle 1. Angle2-angle1
	   * @param angle1
	   * @param angle2
	   * @return
	   */
	  public float getAngleChange(float angle1, float angle2){
		  float angleChange=angle2-angle1;
			if (angleChange>Math.PI){angleChange=(float) (angleChange-2*Math.PI);}
			if (angleChange>Math.PI){angleChange=(float) (angleChange-2*Math.PI);}
			return angleChange;
	  }
	  

	   public static Vector3f getGlobalForce(Vector3f localforce, RigidBody body){
		    Transform t=new Transform();
			t=body.getMotionState().getWorldTransform(t);
			Vector3f globalForce=new Vector3f(0,0,0);
			globalForce.x=localforce.dot(new Vector3f(t.basis.m00, t.basis.m10, t.basis.m20));
			globalForce.y=localforce.dot(new Vector3f(t.basis.m01, t.basis.m11, t.basis.m21));
			globalForce.z=1;//localforce.dot(new Vector3f(t.basis.m02, t.basis.m12, t.basis.m22));
	        return globalForce;
	        }
	   
	   
	   
		private void drawLine(Vector3f from, Vector3f to,Vector3f color) {
			gl.glBegin(GL_LINES);
			gl.glColor3f(color.x, color.y, color.z);
			gl.glVertex3f(from.x, from.y, from.z);
			gl.glVertex3f(to.x, to.y, to.z);
			gl.glEnd();
		}
	   
		
		public static float[][] getCaseData(int caseCount, int caseNum, float[][][] trackingData){
			float[][] caseData= new float[caseCount][3];
		    for (int i=0;i<caseCount;i++){
				for(int j1=0; j1<=2;j1++){
					caseData[i][j1]=trackingData[i][j1][caseNum];
					//System.out.println("caseData["+i+"]["+j1+"]="+caseData[i][j1]);
				}
		     }
		    return caseData;
		    }
	}
