package com.bulletphysics;

import static com.bulletphysics.demos.opengl.IGL.GL_LINES;
import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.InternalTickCallback;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;

/**
 * Added soil building.
 * Model the building/digging behavior.
 * 1. Termites start digging the soil when they see an exiting digging site on the front(with some probability).
 * 2. A dig is consider successful if termites unmoved for 3 sec(15 frame).
 * 3. Termites deposit soil when they see an existing depositing site on the front.
 * (at each step, determine if deposit by drawing a random number and testing if the number is larger than 0.8. If yes, deposit. If not, do not deposit.).

 *
 */
public class Model3 extends InternalTickCallback{	
	private static int range=18;//how far away the termite could sense
	private static float termiteHalfLen;
	private static int[] values=new int[4];
	private static double angleRange=Math.PI/2;
	private static float dishRadius=BuildingDemo.getDishRadius();
    private static int digDepositDuration=20;
	public static int[] inputDistribution= new int[27];
	private int pullDownForce=5;
	private static double randomStart=0.98;
	private static double randomEnd=0.9;
	private DynamicsWorld dynamicsWorld;
	private IGL gl;
	private static boolean applyNew=false;
	public static int[] getInputDis(){return inputDistribution;}
   public static  int[] returnInputDis(){return inputDistribution;}
	
	
	public Model3(DynamicsWorld dy, IGL gl) {
		this.dynamicsWorld=dy;
		this.gl=gl;
	}
	

	public void internalTick(DynamicsWorld dynamicsWorld, float timeStep) {	
		ObjectArrayList<RigidBody> termites= BuildingDemo.getTermites();
		ArrayList<ArrayList<Float>> posList=BuildingDemo.getPositionList();
        
		//record the position every 1/5 sec
		int count=BuildingDemo.getCount();
		Long diff=(long)300;
		long time=BuildingDemo.getTime();
		applyNew=false;
		if(time<count*200+diff && time>count*200-diff){
			BuildingDemo.incrementCount();
			applyNew=true;
		  	System.out.println("Increment count to "+ count + " at time "+ time/1000);
		  	//for every termite, record the position
			 for (int j=0; j<termites.size(); j++) {
			    	RigidBody body= termites.get(j);	
					Vector3f position= new Vector3f(0,0,0);
					//get the position and orientation of each termite                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
					position=body.getCenterOfMassPosition(position);
					Quat4f orientation=new Quat4f();
					orientation=body.getOrientation(orientation);
					float center_x=position.x;
					float center_y=position.y;
					float angle=getAngle(orientation);
					float terLen=BuildingDemo.getTermiteLen();
					termiteHalfLen=(terLen/2);
					float head_x=(float) (center_x+termiteHalfLen*Math.cos(angle));
					float head_y=(float) (center_y+termiteHalfLen*Math.sin(angle));
					float tail_x=(float) (center_x-termiteHalfLen*Math.cos(angle));
					float tail_y=(float) (center_y-termiteHalfLen*Math.sin(angle));
				  	posList.get(j).add(head_x);	posList.get(j).add(head_y);	posList.get(j).add(tail_x);	posList.get(j).add(tail_y);	
			 }
		}
				
		 // for every termite		
		 for (int j=0; j<termites.size(); j++) {
	    	RigidBody body= termites.get(j);	
			Vector3f position= new Vector3f(0,0,0);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
			position=body.getCenterOfMassPosition(position);
			Quat4f orientation=new Quat4f();
			orientation=body.getOrientation(orientation);	
			float center_x=position.x;
			float center_y=position.y;
			float angle=getAngle(orientation);
			float terLen=BuildingDemo.getTermiteLen();
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
			
			float forcex=(float) (30-(count/1000.0)*4);
			//add some randomness
			Vector3f localforce=new Vector3f(forcex+(float) (Math.random()*10-5),(float) ((float) (Math.random()*10-5)),pullDownForce);
		
			if(applyNew){
					int caseNum=getCaseNum(j);
					inputDistribution[caseNum]+=1;
					//int curState=BuildingDemo.getState(j);
					//Vector3f localforce=getForce(curState,caseNum,caseCount);	
					int nextState=changeState(j,position, head_x,head_y);
					BuildingDemo.setState(nextState, j);
			     	Quat4f rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, 4);
					Transform tr=new Transform();
					tr=body.getCenterOfMassTransform(tr);
					float rotatedAngle=(float) ((float)5+count/1000*4);   //0.15-0.07, approximately 10 degree. Want:0.15,8 degree
					//0.1:2.94255;0.125:2.89188, 0.25: 2.6516; 0.5:2.214; 0.75:1.854,1:1.57, 1.25:1.35; 1.5:1.176; 
			     	//2:0.9272; 2.5:0.76;(45)5:0.394; 10:0.2; 20:0.1; 30:0.0666;40:0.05;50:0.04  
					boolean rotate=false;
			    	double random=Math.random()*100;
			    	double cut=0;
			    	double r=Math.random();
					//first, check for front .inverse turn left
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
						if(values[3]!=0 && values[1]!=0){localforce=new Vector3f(-5,0,pullDownForce);}
					}
			       
			    	else{ //when the front is empty
			    		 //right not empty, left empty, turn to left a bit
						 if ( values[3]==2 && values[1]==0 ){ rotate=true;rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle);rotation.inverse();  }	
						 if ( values[3]==1 && values[1]==0 && random>cut){ rotate=true;rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle);rotation.inverse();  }	
						 //left not empty, right empty, turn to right a bit
						 if ( values[1]==2 && values[3]==0 ){rotate=true;rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle);}  //if nothing on the right, turn right:positive
						 if ( values[1]==1 && values[3]==0 && random>cut){rotate=true;rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle);}  //if nothing on the right, turn right:positive
						 if ( values[3]!=0 && values[1]!=0){localforce=new Vector3f(-5,0,pullDownForce);/*go back*/ }
			    	
		            }
					if(rotate){
							localforce=new Vector3f(0,0,pullDownForce);
							Quat4f newAngle=new Quat4f();
							newAngle=body.getOrientation(newAngle);
							rotation.mul(newAngle);
						    tr.setRotation(rotation);
					        body.setCenterOfMassTransform(tr);
				      }
			     }
				 Vector3f globalForce=rotate(angle,localforce);
				 body.setLinearVelocity(globalForce);	
		    }
	}
	
	private int changeState(int termite, Vector3f position, float head_x, float head_y) {
		int currentState=BuildingDemo.getState(termite);
		int nextState=currentState;
		ObjectArrayList<RigidBody> termites= BuildingDemo.getTermites();
		//if in state 0(moving), start dig if drawing a random number that is larger than randomStart;
		if (currentState==0){
			int inStateZero=BuildingDemo.getTimeInState(termite)[0];
			if (Math.random()>randomStart){
				nextState=1;
				//dig, change mesh height
				Construction.dig(termites.get(termite), position,head_x,head_y,dynamicsWorld);
				BuildingDemo.clearTimeInState(termite);
				BuildingDemo.setTimeInState(termite,1,1);
			}
			else{
				int dur=BuildingDemo.getTimeInState(termite)[0];
				dur+=1;
			    BuildingDemo.setTimeInState(termite,0,dur);
			}
		}
		
		//if in state 1(digging), start moving soil if have been dig for digDepositDuration
		if (currentState==1){
			int inStateOne=BuildingDemo.getTimeInState(termite)[1];
			if (inStateOne>=digDepositDuration){
				nextState=2;
				//dig, change mesh height
				BuildingDemo.clearTimeInState(termite);
				BuildingDemo.setTimeInState(termite,2,1);
			}
			else{
				int dur=BuildingDemo.getTimeInState(termite)[1];
				dur+=1;
				BuildingDemo.setTimeInState(termite,1,dur);
			}
		}
		
		//if in state 2(moving), deposit with random chance
		if (currentState==2){
			int inStateTwo=BuildingDemo.getTimeInState(termite)[2];
			if (Math.random()>randomEnd){
				nextState=3;
				//dig, change mesh height
				Construction.deposit(termites.get(termite),  head_x,head_y,dynamicsWorld);
				BuildingDemo.clearTimeInState(termite);
				BuildingDemo.setTimeInState(termite,3,1);
			}
			else{int dur=BuildingDemo.getTimeInState(termite)[2];
			dur+=1;
			BuildingDemo.setTimeInState(termite,2,dur);}
		}
		
		//if in state 3(depositing), start moving again if have been dig for digDepositDuration
		if (currentState==3){
			int inStateThree=BuildingDemo.getTimeInState(termite)[3];
			if (inStateThree>=digDepositDuration){
				nextState=0;
				//deposit, change mesh height
				BuildingDemo.clearTimeInState(termite);
				BuildingDemo.setTimeInState(termite,0,1);
			}
			else{
				int dur=BuildingDemo.getTimeInState(termite)[3];
				dur+=1;
				BuildingDemo.setTimeInState(termite,3,dur);
			}
		}
		//System.out.println("Next state: "+ nextState);
		return nextState;
	}


	private int getCaseNum(int j) {
	    	RigidBody body= BuildingDemo.getTermites().get(j);	
			Vector3f position= new Vector3f(0,0,0);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
			position=body.getCenterOfMassPosition(position);
			Quat4f orientation=new Quat4f();
			orientation=body.getOrientation(orientation);
			float center_x=position.x;
			float center_y=position.y;
			float angle=getAngle(orientation);
			float terLen=BuildingDemo.getTermiteLen();
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
	             if (Math.hypot(point_x,point_y)>dishRadius){ values[dir]=1;}
			}	 		
			//2.check for other termites
			for (int otherTer=0; otherTer<BuildingDemo.getTermiteCount();otherTer++){
				if (j!=otherTer){
					//First, calculate the distance between two termites(center of the other termite- head position of this termite)
					Vector3f other_position= new Vector3f(0,0,0);
					RigidBody other_body=BuildingDemo.getTermites().get(otherTer);
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
					if ( Math.hypot((other_x-head_x),(other_y-head_y))<range || Math.hypot((other_head_x-head_x),(other_head_y-head_y))<range || Math.hypot((other_tail_x-head_x),(other_tail_y-head_y))<range){
						//if the termite is close, check in which quadrant it is.
						float dis_angle=(float) Math.atan2(other_y-head_y,other_x-head_x);
						float angleChange=getAngleChange(angle, dis_angle);
						int direction=getDirectionFromAngle((double)angleChange);
						values[direction]=2;				
					}
				}
			}
			int caseNum=values[0]+values[1]*3+values[3]*3*3;
			return caseNum;
	}


	private Vector3f getGlobalForce(float angle, Vector3f v){	      
			Double originalAngle=Math.atan2(v.y,v.x);
		    Double newAngle=angle+originalAngle;
		    Float len= (float) Math.sqrt(v.x*v.x+v.y*v.y);
		    float x= (float) ((float)len*Math.cos(newAngle));
		    float y=(float) ((float)len*Math.sin(newAngle));
		    return new Vector3f(x,y,v.z);
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
      * front is 0, left is 1, back is 2, right is 3.
      * @param angle
      * @return
      */
	private int getDirectionFromAngle(double angle) {
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
