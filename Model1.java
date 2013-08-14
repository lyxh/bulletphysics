package com.bulletphysics;

import static com.bulletphysics.demos.opengl.IGL.GL_LINES;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

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
 * The model based on inspection and simple movement rules.
 * @author yixin :)   U_U
 *
 */
public class Model1 extends InternalTickCallback{	
	private static int range=18;//how far away the termite could sense
	private static float termiteHalfLen;
	private static int[] values=new int[4];
	private static double angleRange=Math.PI/2;
	private static float dishRadius=BasicDemo.getDishRadius();
	public static int[] inputDistribution= new int[27];
	private DynamicsWorld dynamicsWorld;
	private IGL gl;
	private float forcex;
	boolean draw=true;
	
	public Model1(DynamicsWorld dy, IGL gl) {
		this.dynamicsWorld=dy;
		this.gl=gl;
	}
    public static int counti;
	public static int[] getInputDis(){return inputDistribution;}
  
	public void internalTick(DynamicsWorld dynamicsWorld, float timeStep) {	
		ObjectArrayList<RigidBody> termites= BasicDemo.getTermites();
		ArrayList<ArrayList<Float>> posList=BasicDemo.getPositionList();
		boolean increased=false;
		int count=BasicDemo.getCount();
		Long diff=(long)100;
		long time=BasicDemo.getTime();
		if(time<count*200+diff && time>count*200-diff){
			counti=BasicDemo.getCounti();
			BasicDemo.clearCounti();
			BasicDemo.incrementCount();
			increased=true;
		  	System.out.println("Increment count to "+ count + " at time "+ time/1000);
		}
		else{BasicDemo.increCounti();}
		//draw the termite direction
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
				float termiteRadius=BasicDemo.getTermiteRad();
				float head_x=(float) (center_x+termiteHalfLen*Math.cos(angle));
				float head_y=(float) (center_y+termiteHalfLen*Math.sin(angle));
				float tail_x=(float) (center_x-termiteHalfLen*Math.cos(angle));
				float tail_y=(float) (center_y-termiteHalfLen*Math.sin(angle));
				
			    if(draw){
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
		         } 
		         if(increased){
		        	 //adding the termite position
			        posList.get(j).add(head_x);	posList.get(j).add(head_y);	posList.get(j).add(tail_x);	posList.get(j).add(tail_y);
					//getting the input case number
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
								float dis_angle=(float) Math.atan2(other_head_y-head_y,other_head_x-head_x);
								float angleChange=getAngleChange(angle, dis_angle);
								int direction=getDirectionFromAngle((double)angleChange);
								values[direction]=2;	
								float dis_angle2=(float) Math.atan2(other_y-head_y,other_x-head_x);
								float angleChange2=getAngleChange(angle, dis_angle2);
								int direction2=getDirectionFromAngle((double)angleChange2);
								values[direction2]=2;
								
								float dis_angle3=(float) Math.atan2(other_tail_y-head_y,other_tail_x-head_x);
								float angleChange3=getAngleChange(angle, dis_angle3);
								int direction3=getDirectionFromAngle((double)angleChange3);
								values[direction3]=2;	
							}
						}
					}
					int caseNum=values[0]+values[1]*3+values[3]*3*3;
					//System.out.println(caseNum);
					inputDistribution[caseNum]+=1;
					Random a= new Random();
					forcex=(float)(a.nextDouble()*75-16.5);//Block 2: hard to get distribution OK.
					//Block 1: 65 -7.5 0.21, 60-5, really good! 70-10  only 0.01
					//could also remember all the random numbers. System.out.println(forcex);
					if(forcex<0){forcex=0;}		//maybe need this?		
					Vector3f localforce=new Vector3f(forcex,0,5);
					Quat4f rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, 4);
					Transform tr=new Transform();
					tr=body.getCenterOfMassTransform(tr);
					float rotatedAngle=((float)3.5);   //0.15-0.07, approximately 10 degree. Want:0.15,8 degree
					float angle2=((float)3.5); 
				    //	rotatedAngle=rotatedAngle/BasicDemo.getConti();
					//0.1:2.94255;0.125:2.89188, 0.25: 2.6516; 0.5:2.214; 0.75:1.854,1:1.57, 1.25:1.35; 1.5:1.176; 
			     	//2:0.9272; 2.5:0.76;(45)5:0.394; 10:0.2; 20:0.1; 30:0.0666;40:0.05;50:0.04  
					//Need to follow the wall more!
					boolean rotate=false;
					double cut=0.88;
					double r=Math.random();
					double random=Math.random();
					//first, check for front .inverse turn left
			        if (values[0]!=0 ){
						if (values[3]==0 || values[1]==0 ){			
			                
			                if (values[3]==0  && random>cut){rotate=true;rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle); }  //if nothing on the right, turn right:positive
			                if (values[1]==0  && random>cut) { rotate=true;rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle); rotation.inverse();}		 //if nothing on the left, turn left
							if (values[3]==0 && values[1]==0){
								rotate=true;
								if (r>0.5){rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle); rotation.inverse();}
								else{rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle);}
							}
						}
						if(values[3]!=0 && values[1]!=0){localforce=new Vector3f(-20,0,5);rotate=false;}
					}
			       
			        else{ //when the front is empty
			    		 //right not empty, left empty, turn to left a bit
						 if ( values[3]!=0 && values[1]==0 ){ rotate=true;rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0,  rotatedAngle);rotation.inverse();  }	
						 //left not empty, right empty, turn to right a bit
						 if ( values[1]!=0 && values[3]==0){rotate=true;rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0,  rotatedAngle);} 
						 if ( values[3]!=0 && values[1]!=0){localforce=new Vector3f(-20, 0,5);rotate=false; }
			    	
		             }	
		           	 
	                 if(rotate){
	                	    //System.out.println("rotate");
							localforce=new Vector3f(0,0,20);
							Quat4f newAngle=new Quat4f();
							newAngle=body.getOrientation(newAngle);
							rotation.mul(newAngle);
						    tr.setRotation(rotation);
					        body.setCenterOfMassTransform(tr);
				     }
	              
	             		                 
						     BasicDemo.randomNums.add((double) forcex);
							 Vector3f globalForce=rotate(angle,localforce);
							 globalForce.z=10;
							 body.setLinearVelocity(globalForce);
							 body.setAngularVelocity(rotate(angle,new Vector3f(0,0,0)));// prevent from rotation here : prevent self-spinning if colliding!
	                 
				}//end of if(increased)
	     }//end of for each termite
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
	 
	
	
     /**
      * front is 0, left is 1, back is 2, right is 3.
      * @param angle
      * @return
      */
	private int getDirectionFromAngle(double angle) {
		 int result=0;
		 double cut=Math.PI/4;
		 if (angle<cut && angle>-cut){result=0;}
		 if (angle<Math.PI/4*3 && angle>cut){result=3;}
		 if (angle<-cut && angle>-Math.PI/4*3){result=1;}
		 if (angle<-Math.PI/4*3 || angle>Math.PI/4*3){result=2;}
		 return result;
	}

	

	/**
	  * Return the angle of the termite, given the quaternion
	  * @param a quaternion that represents the orientation of the termite
	  * @return the angle lying on the xy plane.
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
	   * @return Angle2-angle1, in the range of [-pi,pi]
	   */
	  public float getAngleChange(float angle1, float angle2){
		  float angleChange=angle2-angle1;
			if (angleChange>Math.PI){angleChange=(float) (angleChange-2*Math.PI);}
			if (angleChange>Math.PI){angleChange=(float) (angleChange-2*Math.PI);}
			return angleChange;
	  }
	  
 
	   
	}

