package com.bulletphysics;

import static com.bulletphysics.demos.opengl.IGL.GL_LINES;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.InternalTickCallback;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.BasicDemo;



/**
 * The simplest model that let the termite walk straight forward. When there is a wall on the way, move to the side where there is no obstacle.
 * @author yixin :)   U_U
 *
 */
public class Model1 extends InternalTickCallback{	
	private static int range=20;//how far away the termite could sense
	private static float termiteHalfLen;
	private static float termiteRadius;
	private static int[] values=new int[4];
	private static double angleRange=Math.PI/2;
	private static float dishRadius=BasicDemo.getDishRadius();
	private static float small_dis=5;
	private static float large_dis=20;
	//private int[][] distribution= readDistributionData();
	public static int[] inputDistribution= new int[27];
	private DynamicsWorld dynamicsWorld;
	private IGL gl;
	private float forcex;
	private static int continuing=1;
	
	public Model1(DynamicsWorld dy, IGL gl) {
		this.dynamicsWorld=dy;
		this.gl=gl;
	}

	public static int[] getInputDis(){return inputDistribution;}
  
	public void internalTick(DynamicsWorld dynamicsWorld, float timeStep) {	
		ObjectArrayList<RigidBody> termites= BasicDemo.getTermites();
		ArrayList<ArrayList<Float>> posList=BasicDemo.getPositionList();
		boolean increased=false;
		//record the position every 1/10 sec, 2 times faster.
		//problem with the beginning
		int count=BasicDemo.getCount();
		Long diff=(long)60;
		BasicDemo.incrementCounti();
		long time=BasicDemo.getTime();
		//System.out.println("Time: "+time+"; Count:"+ count);
		if(time<count*200+diff && time>count*200-diff){
			BasicDemo.incrementCount();
			BasicDemo.setConti(0);
			increased=true;
		  	System.out.println("Increment count to "+ count + " at time "+ time/1000);
		  	//for every termite, record the position
			 for (int j=0; j<termites.size(); j++) {
			    	RigidBody body= termites.get(j);	
					Vector3f position= new Vector3f(0,0,0);
					//get the position and orientation of each termite                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
					position=body.getCenterOfMassPosition(position);
					Quat4f orientation=new Quat4f();
					orientation=body.getOrientation(orientation);
					//for each termite, classify the input condition
					float center_x=position.x;
					float center_y=position.y;
					float angle=getAngle(orientation);
				    //position, angle correct 
					float terLen=BasicDemo.getTermiteLen();
					termiteHalfLen=(terLen/2);
					termiteRadius=BasicDemo.getTermiteRad();
					float head_x=(float) (center_x+termiteHalfLen*Math.cos(angle));
					float head_y=(float) (center_y+termiteHalfLen*Math.sin(angle));
					float tail_x=(float) (center_x-termiteHalfLen*Math.cos(angle));
					float tail_y=(float) (center_y-termiteHalfLen*Math.sin(angle));
				  	posList.get(j).add(head_x);	posList.get(j).add(head_y);	posList.get(j).add(tail_x);	posList.get(j).add(tail_y);
				  	
			 }
		}
		
	 
	   for (int j=0; j<termites.size(); j++) {
	    	RigidBody body= termites.get(j);	
			Vector3f position= new Vector3f(0,0,0);
			//get the position and orientation of each termite                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
			position=body.getCenterOfMassPosition(position);
			Quat4f orientation=new Quat4f();
			orientation=body.getOrientation(orientation);
			//for each termite, classify the input condition
			float center_x=position.x;
			float center_y=position.y;
			float angle=getAngle(orientation);
			float terLen=BasicDemo.getTermiteLen();
			termiteHalfLen=(terLen/2);
			termiteRadius=BasicDemo.getTermiteRad();
			float head_x=(float) (center_x+termiteHalfLen*Math.cos(angle));
			float head_y=(float) (center_y+termiteHalfLen*Math.sin(angle));
			float tail_x=(float) (center_x-termiteHalfLen*Math.cos(angle));
			float tail_y=(float) (center_y-termiteHalfLen*Math.sin(angle));
			for (int dir=0;dir<4;dir++){
				values[dir]=0;
	             double point_x=head_x+range*Math.cos(angle-angleRange*dir);
	             double point_y=head_y+range*Math.sin(angle-angleRange*dir);
	             if ((point_x*point_x+point_y*point_y)>(dishRadius)*(dishRadius)){ values[dir]=1;}
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
			
			//draw the termite direction
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


					int caseNum=values[0]+values[1]*3+values[3]*3*3;
					inputDistribution[caseNum]+=1;
   
					forcex=(float) (30-(count/1000.0)*4);
					//add some randomness
					Vector3f localforce=new Vector3f(forcex+(float) (Math.random()*10-5),(float) ((float) (Math.random()*10-5)),5);
					// if ( values[3]!=0 && values[1]!=0 && values[0]!=0){localforce=new Vector3f(-5,0,5);/*go back*/ }
					 if(increased){
					     	Quat4f rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, 4);
							Transform tr=new Transform();
							tr=body.getCenterOfMassTransform(tr);
							float rotatedAngle=(float) ((float)5+count/1000*4);   //0.15-0.07, approximately 10 degree. Want:0.15,8 degree
						//	rotatedAngle=rotatedAngle/BasicDemo.getConti();
							//0.1:2.94255;0.125:2.89188, 0.25: 2.6516; 0.5:2.214; 0.75:1.854,1:1.57, 1.25:1.35; 1.5:1.176; 
					     	//2:0.9272; 2.5:0.76;(45)5:0.394; 10:0.2; 20:0.1; 30:0.0666;40:0.05;50:0.04  
							boolean rotate=false;
					    	double random=Math.random()*100;
					    	double cut=0;
					    	double r=Math.random();
							//first, check for front .inverse turn left
					    	//TODO: get rid of getting stuck
					       if (values[0]!=0 ){
								if (values[3]==0 || values[1]==0 ){
					                rotate=true;
					                if (values[3]==0 ){rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle); }  //if nothing on the right, turn right:positive
					                if (values[1]==0 ){ rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle); rotation.inverse();}		 //if nothing on the left, turn left
									if (values[3]==0 && values[1]==0){
										if (r>0.5){rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle); rotation.inverse();}
										else{rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle);}
									}
								}
								if(values[3]!=0 && values[1]!=0){localforce=new Vector3f(-5,0,5);}
							}
					       
					    	else{ //when the front is empty
					    		 //right not empty, left empty, turn to left a bit
								 if ( values[3]==2 && values[1]==0 ){ rotate=true;rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle);rotation.inverse();  }	
								 if ( values[3]==1 && values[1]==0 && random>cut){ rotate=true;rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle);rotation.inverse();  }	
								 //left not empty, right empty, turn to right a bit
								 if ( values[1]==2 && values[3]==0 ){rotate=true;rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle);}  //if nothing on the right, turn right:positive
								 if ( values[1]==1 && values[3]==0 && random>cut){rotate=true;rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle);}  //if nothing on the right, turn right:positive
								 
								 if ( values[3]!=0 && values[1]!=0){localforce=new Vector3f(-5,0,5);/*go back*/ }
					    	
				          }
				         
							if(rotate){
									localforce=new Vector3f(0,0,5);
									Quat4f newAngle=new Quat4f();
									newAngle=body.getOrientation(newAngle);
									rotation.mul(newAngle);
								    tr.setRotation(rotation);
							        body.setCenterOfMassTransform(tr);
		
						    }
                   }
					Vector3f globalForce=rotate(angle,localforce);
					//drawLine(position,new Vector3f(position.x+globalForce.x,position.y+globalForce.y,position.z ),new Vector3f(1,1,0));
					body.setLinearVelocity(globalForce);			       
           }
		}
	
	
	private void drawLine(Vector3f from, Vector3f to,Vector3f color) {
		gl.glBegin(GL_LINES);
		gl.glColor3f(color.x, color.y, color.z);
		gl.glVertex3f(from.x, from.y, from.z);
		gl.glVertex3f(to.x, to.y, to.z);
		gl.glEnd();
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
		 //first, generate the angle turned
		float angle = 0;
		float angleCaseNum=caseData[0]+caseData[1]+caseData[2];
		float random=(float) (Math.random());
		if (random<caseData[0]/angleCaseNum){angle=0;}
		if (random>=caseData[0]/angleCaseNum && random<(caseData[0]+caseData[0])/angleCaseNum){angle=90;} //turn left
		else{angle=-90;} //turn right
		
		//generate x and y distances
		float x=0;
		float xCaseNum=caseData[3]+caseData[4]+caseData[5]+caseData[6]+caseData[7];
		float random_x=(float) (Math.random());
		float one=caseData[3]/xCaseNum;
		float two=(caseData[3]+caseData[4])/xCaseNum;
		float three=(caseData[3]+caseData[4]+caseData[5])/xCaseNum;
		float four=(caseData[3]+caseData[4]+caseData[5]+caseData[6])/xCaseNum;
		//System.out.println(one+" "+two+" "+three+" "+four);
		if (random_x<one){x=0;}
		else if (random_x>=one && random_x<two){x=small_dis;} 
		else if (random_x>=two && random_x<three){x=large_dis;} 
		else if (random_x>=three && random_x<four){x=-small_dis;} 
		else if (random_x>=four){x=-large_dis;} 
		
		
		float y=0;
		float yCaseNum=caseData[8]+caseData[9]+caseData[10]+caseData[11]+caseData[12];
		float random_y=(float) (Math.random());
		float one_y=caseData[8]/yCaseNum;
		float two_y=(caseData[8]+caseData[9])/yCaseNum;
		float three_y=(caseData[8]+caseData[9]+caseData[10])/yCaseNum;
		float four_y=(caseData[8]+caseData[9]+caseData[10]+caseData[11])/yCaseNum;
		if (random_y<one_y){y=0;}
		else if (random_y>=one_y && random_y<two_y){y=small_dis;} 
		else if (random_y>=two_y && random_y<three_y){y=large_dis;} 
		else if (random_y>=three_y && random_y<four_y){y=-1*small_dis;} 
		else if (random_y>=four_y){y=-1*large_dis;} 

		// Now, without rotation.
		Float[] newxy=rotate(angle,x,y);
		//from angle,x,y,generate a force
		//Vector3f force=new Vector3f((float)newxy[0],(float)newxy[1],(float)0);
		//how to rotate the rigidbody?
		Vector3f force=new Vector3f(x,y,0);
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
	 
	 
	 public Vector3f rotate(float angle,Vector3f v){
	      Double originalAngle=Math.atan2(v.y,v.x);
	      Double newAngle=angle+originalAngle;
	      Float len= (float) Math.sqrt(v.x*v.x+v.y*v.y);
	      float x= (float) ((float)len*Math.cos(newAngle));
	      float y=(float) ((float)len*Math.sin(newAngle));
	      return new Vector3f(x,y,v.z);
	   }
	 
   /**
 * @return 
    * 
    */
	private int[][] readDistributionData() {
		int[][] result= new int[13][27];
		 String filePath="D:\\Yixin\\trajectory analysis\\PherDish11Block1.txt";
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
	  public static float getAngle(Quat4f orientation){
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
			globalForce.x=localforce.dot(new Vector3f(t.basis.m00, t.basis.m10, 0));
			globalForce.y=localforce.dot(new Vector3f(t.basis.m01, t.basis.m11, 0));
			globalForce.z=1;//localforce.dot(new Vector3f(t.basis.m02, t.basis.m12, t.basis.m22));
	        return globalForce;
	        }
	   
	}

