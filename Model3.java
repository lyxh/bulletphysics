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
public class Model3 extends InternalTickCallback{	
	private static int range=30;//how far away the termite could sense
	private static float termiteHalfLen;
	private static int[] values=new int[4];
	private static double angleRange=Math.PI/2;
	private static float dishRadius=BasicDemo.getDishRadius();
	private static float small_dis=10;
	private static float large_dis=50;
	private static String filePath="D:\\Yixin\\model\\Case_Avg_Data_Model_2.txt";
	private float[][] distribution= new float[3][27];
	public static int[] inputDistribution= new int[27];
	private int pullDownForce=1;
	private DynamicsWorld dynamicsWorld;
	private IGL gl;
	private static int continuing=5;
	
	public static int[] getInputDis(){return inputDistribution;}
	
	
	public Model3(DynamicsWorld dy, IGL gl) {
		this.dynamicsWorld=dy;
		this.gl=gl;
		this.distribution=readDistributionData(filePath);
	}
	

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
			
			//Get the input case number
			int caseNum=values[0]+values[1]*3+values[3]*3*3;
			//System.out.println("Case "+caseNum);
			Vector3f localforce=new Vector3f(distribution[0][caseNum]*2,distribution[1][caseNum]*4,0);
			
			//System.out.println(localforce);
			float rotatedAngle=-distribution[2][caseNum]*50;
			Transform tr=new Transform();
			tr=body.getCenterOfMassTransform(tr);
		    tr.basis.rotZ(rotatedAngle);	    
		    body.setCenterOfMassTransform(tr);
		    
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
			localforce=rotate(rotatedAngle, localforce);
			Vector3f globalForce=rotate(angle,localforce);
			drawLine(position,new Vector3f(position.x+globalForce.x*3,position.y+globalForce.y*3,position.z ),new Vector3f(1,1,0));
			
			
			globalForce.z=pullDownForce;
			body.setLinearVelocity(globalForce);
	    }
	}
	
	
	private float getAngleFromDistribution(int[] caseData) {
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
 * @return 
    * 
    */
	private float[][] readDistributionData(String filePath) {
		float[][] result= new float[3][27];
		 byte[] buffer = new byte[(int) new File(filePath).length()];
		    BufferedInputStream f = null;
		    try {f = new BufferedInputStream(new FileInputStream(filePath));
		        f.read(buffer);
		        if (f != null) try { f.close(); } catch (IOException ignored) { }} 
	        catch (IOException ignored) { System.out.println("File not found or invalid path.");}			    
		    String[] strings=(new String(buffer)).split("\\s+");
		    for (int i=0; i<strings.length;i++){
		    	BigDecimal number = new BigDecimal(strings[i]);
		    	  String numWithNoExponents = number.toPlainString();
		    	  float num=Float.valueOf(numWithNoExponents);
		    	  int mod=i%3;
		    	  int div=i/3;
		          result[mod][div]=num;
		          //System.out.println("result["+mod +"]["+div+"]="+result[mod][div]);
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
		
		
		 public Vector3f rotate(float angle,Vector3f v){
		      Double originalAngle=Math.atan2(v.y,v.x);
		      Double newAngle=angle+originalAngle;
		      Float len= (float) Math.sqrt(v.x*v.x+v.y*v.y);
		      float x= (float) ((float)len*Math.cos(newAngle));
		      float y=(float) ((float)len*Math.sin(newAngle));
		      return new Vector3f(x,y,v.z);
		   }
	}
