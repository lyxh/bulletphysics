package com.bulletphysics;

import static com.bulletphysics.demos.opengl.IGL.GL_LINES;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.math.BigDecimal;
import java.util.ArrayList;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.InternalTickCallback;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.BasicDemo2;

/**
 * The model that is similar to model 1. Only forward, side distance and angle rotated are taken into account
 * But the distribution is drawn from the original data set.
 * @author ssr
 *
 */
public class Model2 extends InternalTickCallback{	
	private static int range=18;//how far away the termite could sense
	private static float termiteHalfLen;
	private static int[] values=new int[4]; 
	private static double angleRange=Math.PI/2;
	private static float dishRadius=BasicDemo2.getDishRadius();
    public static int[] inputDistribution= new int[27];
	private int pullDownForce=5;
	private DynamicsWorld dynamicsWorld;
	private IGL gl;
	private static boolean applyNew=false;
	public static int[] getInputDis(){return inputDistribution;}
    public static  int[] returnInputDis(){return inputDistribution;}
	public boolean draw=true;
    /**
     * Constructor
     * @param dy
     * @param gl
     */
	public Model2(DynamicsWorld dy, IGL gl) {
		this.dynamicsWorld=dy;
		this.gl=gl;
	}
	
    /**
     * Called during every internal click of BasicDemo2
     */
	public void internalTick(DynamicsWorld dynamicsWorld, float timeStep) {	
		ObjectArrayList<RigidBody> termites= BasicDemo2.getTermites();
		ArrayList<ArrayList<Float>> posList=BasicDemo2.getPositionList();
		int count=BasicDemo2.getCount();
		Long diff=(long)50;
		long time=BasicDemo2.getTime();
		applyNew=false;
		
		//if the simulation elapse by 0.2 sec(+-0.05 sec), need to record the position and apply the controller.
		if(time<count*200+diff && time>count*200-diff){
			BasicDemo2.incrementCount();
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
					float terLen=BasicDemo2.getTermiteLen();
					termiteHalfLen=(terLen/2);
					float head_x=(float) (center_x+termiteHalfLen*Math.cos(angle));
					float head_y=(float) (center_y+termiteHalfLen*Math.sin(angle));
					float tail_x=(float) (center_x-termiteHalfLen*Math.cos(angle));
					float tail_y=(float) (center_y-termiteHalfLen*Math.sin(angle));
					 //record the position roughly every 1/5 sec
				  	posList.get(j).add(head_x);	posList.get(j).add(head_y);	posList.get(j).add(tail_x);	posList.get(j).add(tail_y);	
			 }
		}
		
		
		 for (int j=0; j<termites.size(); j++) {
	    	RigidBody body= termites.get(j);	
	    	//get the position and orientation of each termite    
			Vector3f position= new Vector3f(0,0,0);
			position=body.getCenterOfMassPosition(position);
			Quat4f orientation=new Quat4f();
			orientation=body.getOrientation(orientation);	
			
			float center_x=position.x;
			float center_y=position.y;
			float angle=getAngle(orientation);
			float terLen=BasicDemo2.getTermiteLen();
			termiteHalfLen=(terLen/2);
			float head_x=(float) (center_x+termiteHalfLen*Math.cos(angle));
			float head_y=(float) (center_y+termiteHalfLen*Math.sin(angle));
			float tail_x=(float) (center_x-termiteHalfLen*Math.cos(angle));
			float tail_y=(float) (center_y-termiteHalfLen*Math.sin(angle));
					
			//draw the termites sensing range
			if (draw){
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
			
			if (applyNew){
			
					//apply the controller
						
					// First, get the input case number
					//1.check for wall.tested
					for (int dir=0;dir<4;dir++){
						values[dir]=0;
			             double point_x=head_x+range*Math.cos(angle-angleRange*dir);
			             double point_y=head_y+range*Math.sin(angle-angleRange*dir);
			             if ((point_x*point_x+point_y*point_y)>dishRadius*dishRadius){ values[dir]=1;}
					}	 
					//2.check for other termites
					for (int otherTer=0; otherTer<BasicDemo2.getTermiteCount();otherTer++){
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
							}
						}
					}
			
					int caseNum=values[0]+values[1]*3+values[3]*3*3;
					//System.out.println(caseNum);
					inputDistribution[caseNum]+=1;
					
					int caseCount=BasicDemo2.getCaseCount()[caseNum];
					
					// draw a sample from the sample base
					Vector3f localforce=getForceAngleFromDistribution(caseNum,caseCount);
				    
					float rr=(float)localforce.z; 
				    
					float rotatedAngle=0;
				    //change rotatedAngle to the right value. (Specified: actually turned)
			     	//0.1:2.94255;0.125:2.89188, 0.25: 2.6516; 0.5:2.214; 0.75:1.854,1:1.57, 1.25:1.35; 1.5:1.176; 
			     	//2:0.9272; 2.5:0.76;(45)5:0.394; 10:0.2; 20:0.1; 30:0.0666;40:0.05;50:0.04  
				    float r=Math.abs(rr);
				    if(r>0 && r<=0.0666){rotatedAngle=35;}
				    else if(r>0.0666 && r<=0.1){rotatedAngle=25;}
				    else if(r>0.1 && r<=0.2){rotatedAngle=15;}
				    else if(r>0.2 && r<=0.4){rotatedAngle=7;}
				    else if(r>0.4 && r<=0.8){rotatedAngle=3;}
				    else if(r>0.8 && r<=0.9){rotatedAngle=(float) 2.2;}
				    else if(r>0.9 && r<=1.176){rotatedAngle=(float) 1.5;}
				    else if(r>1.176 && r<=1.57){rotatedAngle=(float) 1.25;}
				    else if(r>1.57 && r<=2.217){rotatedAngle=(float) 0.75;}
				    else{
				    	rotatedAngle=(float) 0.6;
				    }
				    if(rr<0){rotatedAngle*=-1;}
				    
					localforce.z=10;
		            //System.out.println(rotatedAngle);

					 float newA=angle;
					 //sanity check for if tracking data is correct. Rotating >0.75pi should never happens in tracking data.
					 if (Math.abs(rotatedAngle)>=1){ 	
				            Transform tr=new Transform();
							tr=body.getCenterOfMassTransform(tr);
							Quat4f newAngle=new Quat4f();
							newAngle=body.getOrientation(newAngle);
							if (rotatedAngle>0){ 
								//turn left
								Quat4f rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, rotatedAngle);
								rotation.inverse();
								rotation.mul(newAngle);
							    tr.setRotation(rotation);
						        body.setCenterOfMassTransform(tr); 
						        newAngle=body.getOrientation(newAngle);
							}
							if (rotatedAngle<=0){
								//turn right
								Quat4f  rotation=new Quat4f((float)0.0, (float)0.0, (float)1.0, -rotatedAngle);
								rotation.mul(newAngle);
							    tr.setRotation(rotation);
						        body.setCenterOfMassTransform(tr); 
						        newAngle=body.getOrientation(newAngle);
							}	
							// update the angle of the termite 
							newA=getAngle(newAngle);
		            }
					 
			        //since we already rotated the termite, get the force in global coordinates from the local force and the new orientation of the termite   
					Vector3f globalForce=rotate(newA,localforce);
					globalForce.z=30;
					//drawLine(position,new Vector3f(position.x+globalForce.x,position.y+globalForce.y,position.z ),new Vector3f(1,1,0));
					body.setLinearVelocity(globalForce);
					body.setAngularVelocity(rotate(angle,new Vector3f(0,0,0)));// prevent from rotation here : prevent self-spinning if colliding!
			    }
		 }
	}
	
	

	/**
	 * Given the caseNum, draw a ramdom sample from the sample pool. Multiply the forwar dna sideward speed by 5 since
	 * x and y in the original sample have unit px/0.2sec.
	 * @param caseNum: the input case number
	 * @param caseCount: the time of occurance for case caseNum in the tracking data(one block only)
	 * @return sample in the form of (x_dis,y_dis,angle_rotated)
	 */
	 private synchronized Vector3f getForceAngleFromDistribution(int caseNum,int caseCount) {
        float x=10;
	    float y=0;
	    float angle=0;
		if (caseCount!=0){
		        int random=(int) (Math.random()*(float)caseCount);
		        if(random>=caseCount & (int)caseCount!=0){random=caseCount-1;}
		          x=BasicDemo2.trackingData.get(caseNum)[random][0];
		          y=BasicDemo2.trackingData.get(caseNum)[random][1];
		          angle=BasicDemo2.trackingData.get(caseNum)[random][2];	
		 }
		//5 times the velocity/0.2sec.  
		Vector3f force=new Vector3f(x*5,y*5,angle);
		return force;
	}

	
     /**
      * front is 0, left is 1, back is 2, right is 3.
      * @param angle the angle, in range of [-pi,pi]
      * @return the area the angle is in.
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
	  * Get the angle(parallel to the x-y plane) from a quaternion
	  * @param a quaternion that represents the orientation of the termite
	  * @return anglr
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
		    return euler.y;		  //we want yaw!
	  }

	  
	  /**
	   * Calculate the angle difference between angle 2 and angle 1. 
	   * Note: the order matters! Return Angle2-angle1, in the range of [-pi,pi]
	   * @param angle1 the first angle
	   * @param angle2  the second angle
	   * @return Angle2-angle1, in the range of [-pi,pi]
	   */
	  public float getAngleChange(float angle1, float angle2){
		  float angleChange=angle2-angle1;
			if (angleChange>Math.PI){angleChange=(float) (angleChange-2*Math.PI);}
			if (angleChange>Math.PI){angleChange=(float) (angleChange-2*Math.PI);}
			return angleChange;
	  }
	  
	   
	   
	   /**
	    * Draw a line of some color from start position to the end position
	    * @param from: a vector of the start position
	    * @param to: a vector of the end position
	    * @param color: the color of the line
	    */
		private void drawLine(Vector3f from, Vector3f to,Vector3f color) {
			gl.glBegin(GL_LINES);
			gl.glColor3f(color.x, color.y, color.z);
			gl.glVertex3f(from.x, from.y, from.z);
			gl.glVertex3f(to.x, to.y, to.z);
			gl.glEnd();
		}
	   
		/**
		 * 
		 * @param caseCount
		 * @param caseNum
		 * @param trackingData
		 * @return
		 */
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
		
		/**
		 * 
		 * @param angle
		 * @param v
		 * @return
		 */
		 public Vector3f rotate(float angle,Vector3f v){
		      Double originalAngle=Math.atan2(v.y,v.x);
		      Double newAngle=angle+originalAngle;
		      Float len= (float) Math.sqrt(v.x*v.x+v.y*v.y);
		      float x= (float) ((float)len*Math.cos(newAngle));
		      float y=(float) ((float)len*Math.sin(newAngle));
		      return new Vector3f(x,y,v.z);
		   }
	}
