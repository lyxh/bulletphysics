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
 * The model that use the transition probability from the txt file.
 * At each step, three parameters are independently determined:x,y,angle.
 * Seems to be wrong....
 * @author ssr
 *
 */
public class Model10 extends InternalTickCallback{	
	private static int range=20;//how far away the termite could sense
	private static float termiteHalfLen;
	private static int[] values=new int[4];
	private static double angleRange=Math.PI/2;
	private static float dishRadius=BasicDemo.getDishRadius();
	private static float small_dis=10;
	private static float large_dis=20;
	private static String filePath="D:\\Yixin\\trajectory analysis\\PherDish11Data2.txt";
	private int[][] distribution= readDistributionData(filePath);
	public static int[] inputDistribution= new int[27];
	private int pullDownForce=1;
	private DynamicsWorld dynamicsWorld;
	private IGL gl;
	private static int continuing=5;
	
	public static int[] getInputDis(){return inputDistribution;}
	
	
	public Model10(DynamicsWorld dy, IGL gl) {
		this.dynamicsWorld=dy;
		this.gl=gl;
	}
	
	//TODO: match the video
	//TODO: annotate the assumption
	public void internalTick(DynamicsWorld dynamicsWorld, float timeStep) {	
		ObjectArrayList<RigidBody> termites= BasicDemo.getTermites();
		readDistributionData(filePath);
	    for (int j=0; j<termites.size(); j++) {
	    	RigidBody body= termites.get(j);	
			Vector3f position= new Vector3f(0,0,0);
			//get the position and orientation of each termite                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
			position=body.getCenterOfMassPosition(position);
			Quat4f orientation=new Quat4f();
			orientation=body.getOrientation(orientation);
		//	posList.get(j).add(position);
			//oriList.get(j).add(orientation);
			
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
						//System.out.println("Angle 1: "+angle/Math.PI*180);
						//System.out.println("Angle 2: "+dis_angle/Math.PI*180);
						//System.out.println(angleChange/Math.PI*180);					
						//System.out.println("Direction: " +(direction+1));		
						//Also check for other termite head and tail?			
					}
				}
			}
			
			

			float force=(float) (6*(1-0/5000)*5);
			//Get the input case number
			int caseNum=values[0]+values[1]*3+values[3]*3*3;
			inputDistribution[caseNum]+=1;
			//Get the corresponding distribution
			distribution= readDistributionData(filePath);
			// get the caseData
			int[] caseData= new int[13];
			for (int k=0;k<13;k++){
				caseData[k]=(distribution[k][caseNum]);//checked,correct
			}
			Vector3f localforce=getForceFromDistribution(caseData);
		
			//System.out.println(localforce);
			float rotatedAngle=getAngleFromDistribution(caseData);
			Float[] newxy=rotate(rotatedAngle,localforce.x,localforce.y);
			localforce.x=newxy[0];localforce.y=newxy[1];
			//System.out.println(rotatedAngle);
			Vector3f globalForce=getGlobalForce(localforce,body);
			globalForce.z=1;
			/*
			Transform tr=new Transform();
			tr=body.getCenterOfMassTransform(tr);
		    tr.basis.rotZ(rotatedAngle);
		    body.setCenterOfMassTransform(tr);
			
			*/
			body.setLinearVelocity(globalForce);
	    }
		

		}
	
	
	private float getAngleFromDistribution(int[] caseData) {
		// TODO Auto-generated method stub
		float angle = 0;
		float angleCaseNum=caseData[0]+caseData[1]+caseData[2];
		float random=(float) (Math.random());
		if (random<caseData[0]/angleCaseNum){angle=0;}
		if (random>=caseData[0]/angleCaseNum && random<(caseData[0]+caseData[0])/angleCaseNum){angle=(float) ((float) 1.57/Math.PI*-1);} //turn left
		else{angle=(float) ((float) 1.57/Math.PI*180);} //turn right
		return angle;
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
	 private Vector3f getForceFromDistribution(int[] caseData) {
		//generate x and y distances
		float forward=0;
		float xCaseNum=caseData[3]+caseData[4]+caseData[5]+caseData[6]+caseData[7];
		float random_x=(float) (Math.random());
		float one=caseData[3]/xCaseNum;
		float two=(caseData[3]+caseData[4])/xCaseNum;
		float three=(caseData[3]+caseData[4]+caseData[5])/xCaseNum;
		float four=(caseData[3]+caseData[4]+caseData[5]+caseData[6])/xCaseNum;
		//System.out.println(one+" "+two+" "+three+" "+four);
		if (random_x<one){forward=0;}
		else if (random_x>=one && random_x<two){forward=small_dis;} 
		else if (random_x>=two && random_x<three){forward=large_dis;} 
		else if (random_x>=three && random_x<four){forward=-small_dis;} 
		else if (random_x>=four){forward=-large_dis;} 
		
		
		float side=0;
		float yCaseNum=caseData[8]+caseData[9]+caseData[10]+caseData[11]+caseData[12];
		float random_y=(float) (Math.random());
		float one_y=caseData[8]/yCaseNum;
		float two_y=(caseData[8]+caseData[9])/yCaseNum;
		float three_y=(caseData[8]+caseData[9]+caseData[10])/yCaseNum;
		float four_y=(caseData[8]+caseData[9]+caseData[10]+caseData[11])/yCaseNum;
		if (random_y<one_y){side=0;}
		else if (random_y>=one_y && random_y<two_y){side=small_dis;} 
		else if (random_y>=two_y && random_y<three_y){side=large_dis;} 
		else if (random_y>=three_y && random_y<four_y){side=-1*small_dis;} 
		else if (random_y>=four_y){side=-1*large_dis;} 
		//from angle,x,y,generate a force
		Vector3f force=new Vector3f(forward,side,0);
	//	System.out.println(force);
		return force;
	}

	 
	 
	 /**
	  * Rotate a vector(x,y) by some degrees angle
	  * @param angle
	  * @param x
	  * @param y
	  * @return
	  */
	 public Float[] rotate(float angle,float x,float y){
	      Double originalAngle=Math.atan2(y,x);
	      Double newAngle=angle+originalAngle;
	      Float len= (float) Math.sqrt(x*x+y*y);
	      Float[] result=new Float[2];
	      result[0]= (float) ((float)len*Math.cos(newAngle));
	      result[1]=(float) ((float)len*Math.sin(newAngle));
	      return result;
	   }
	 
   /**
 * @return 
    * 
    */
	private int[][] readDistributionData(String filePath) {
		int[][] result= new int[13][27];

		 byte[] buffer = new byte[(int) new File(filePath).length()];
		    BufferedInputStream f = null;
		    try {f = new BufferedInputStream(new FileInputStream(filePath));
		        f.read(buffer);
		        if (f != null) try { f.close(); } catch (IOException ignored) { }} 
	        catch (IOException ignored) { System.out.println("File not found or invalid path.");}			    
		    String[] strings=(new String(buffer)).split("\\s+");
		    for (int i=0; i<strings.length;i++){
		    	  Integer num=Integer.valueOf(strings[i]);
		    	  int mod=i%13;
		    	  int div=i/13;
		          result[mod][div]=(int) num;
		         // System.out.println("result["+mod +"]["+div+"]="+result[mod][div]);
		   }
		    
		return result;
	}

	
	
     /**
      * front is 0, left is 1, back is 2, right is 3.
      * @param angle in the range of [-pi,pi]
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
	  * Return the angle from the quaternion
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
	   * @return the angle difference in the range of [-pi,pi]
	   */
	  public float getAngleChange(float angle1, float angle2){
		  float angleChange=angle2-angle1;
			if (angleChange>Math.PI){angleChange=(float) (angleChange-2*Math.PI);}
			if (angleChange>Math.PI){angleChange=(float) (angleChange-2*Math.PI);}
			return angleChange;
	  }
	  
       /**
        * Given the body and the force in the body's local coordinates, return the force in the global coordinates.
        * @param localforce
        * @param body
        * @return
        */
	   public static Vector3f getGlobalForce(Vector3f localforce, RigidBody body){
		    Transform t=new Transform();
			t=body.getMotionState().getWorldTransform(t);
			Vector3f globalForce=new Vector3f(0,0,0);
			globalForce.x=localforce.dot(new Vector3f(t.basis.m00, t.basis.m10, t.basis.m20));
			globalForce.y=localforce.dot(new Vector3f(t.basis.m01, t.basis.m11, t.basis.m21));
			globalForce.z=5;//localforce.dot(new Vector3f(t.basis.m02, t.basis.m12, t.basis.m22));
	        return globalForce;
	        }
	   
	   
		private void drawLine(Vector3f from, Vector3f to,Vector3f color) {
			gl.glBegin(GL_LINES);
			gl.glColor3f(color.x, color.y, color.z);
			gl.glVertex3f(from.x, from.y, from.z);
			gl.glVertex3f(to.x, to.y, to.z);
			gl.glEnd();
		}
	}
