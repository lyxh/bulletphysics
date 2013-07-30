package com.bulletphysics;
import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.math.BigDecimal;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.zip.GZIPInputStream;
import java.util.zip.GZIPOutputStream;

import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.collision.shapes.CapsuleShape;
import com.bulletphysics.collision.shapes.CapsuleShapeX;
import com.bulletphysics.collision.shapes.CapsuleShapeZ;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.OptimizedBvh;
import com.bulletphysics.collision.shapes.ScalarType;
import com.bulletphysics.collision.shapes.StaticPlaneShape;
import com.bulletphysics.collision.shapes.TriangleIndexVertexArray;
import com.bulletphysics.demos.dynamiccontrol.TestRig;
import com.bulletphysics.demos.opengl.DemoApplication;
import com.bulletphysics.demos.opengl.GLDebugDrawer;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.demos.opengl.LWJGL;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.InternalTickCallback;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.HingeConstraint;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.Transform;

import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import org.lwjgl.LWJGLException;
import org.lwjgl.util.vector.Matrix3f;
import org.lwjgl.util.vector.Quaternion;

import static com.bulletphysics.demos.opengl.IGL.*;

/**
 */
public class BuildingDemo extends DemoApplication {
	private static final String NUM_TEXTURES = null;
	private static  ObjectArrayList<CollisionShape> collisionShapes = new ObjectArrayList<CollisionShape>();
	private BroadphaseInterface broadphase;
	private CollisionDispatcher dispatcher;
	private ConstraintSolver solver;
	private DefaultCollisionConfiguration collisionConfiguration;
	
	private RigidBody dish;       
	private static int dishRadius=200;
	private int dishPoints=5000;
	private int numOfTriangles=dishPoints;
	private int dishHeight=30;
	
	private static BvhTriangleMeshShape soil;
	private static TriangleIndexVertexArray  soilPoints;
	private int soilHeight=0;
	private static RigidBody soilBody;
	private static ByteBuffer gVertices;
	private static ByteBuffer gIndices;
	private static int NUM_VERTS_X = (1+dishRadius/10)*2;
	private static int NUM_VERTS_Y = NUM_VERTS_X ;
	private static int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
	

	private static ObjectArrayList<RigidBody> termites= new ObjectArrayList<RigidBody>();
	private static int numOfTermites=1;
	private static int[] states=new int[numOfTermites];
	private static ArrayList<int[]> timeSpentInOneState= new ArrayList<int[]>();
	private static float termiteRadius=5;
    private static float termiteLen=26;
    private float termiteHeight=-6;
    private static ArrayList<ArrayList<Float>> positionList= new ArrayList<ArrayList<Float>>();
	private static Long start_time;
 	public static ArrayList<float[][]> trackingData=new ArrayList<float[][]>();
 	private static int count=0;
 	private static int[] caseCount=new int[27];
	private static int counter=0;
	public static ArrayList<Float> soilMeshHeight=new ArrayList<Float>();
	
	
	public BuildingDemo(IGL gl) {super(gl);}  
	public static ObjectArrayList<RigidBody> getTermites(){	return termites;}	
	public static ArrayList<ArrayList<Float>> getPositionList(){return positionList;}	

	public static BvhTriangleMeshShape getSoilMesh(){	return soil;}
	public static void setSoilMesh(BvhTriangleMeshShape newSoil){	soil=newSoil;}
	
	public static RigidBody getSoil(){return soilBody;}
	public static void setSoil( RigidBody newSoilBody){soilBody=newSoilBody;}
	
	public static float getTermiteLen(){return termiteLen;}
	public static float getTermiteRad(){return termiteRadius;}
	public static int getDishRadius(){return dishRadius;}
	public static int getTermiteCount(){return numOfTermites;}
	public static int[] getStates(){return states;}
	public static ArrayList<float[][]>  getData(){return trackingData;}
	public static int[] getCaseCount() {return caseCount;}
	public static int getCounter() {return counter;}

	
	public static int getState(int i){return states[i];}
	public static void setState(int newstate, int i){states[i]=newstate;}
	public static int[] getTimeInState(int termite){return timeSpentInOneState.get(termite);}
	public static void setTimeInState(int termite, int stateNum, int newVal){timeSpentInOneState.get(termite)[stateNum]=newVal;}
	public static void clearTimeInState(int termite){
		for (int state=0; state<=3;state++){timeSpentInOneState.get(termite)[state]=0;}
	}
	//also need to remember the height of the soil
    public static void setHeight(int pointNum, Float newVal){
    	Float old=soilMeshHeight.get(pointNum);
    	old=newVal;
    	}
    public static  ArrayList<Float>  getHeight(){return soilMeshHeight;}
	static ObjectArrayList<CollisionShape> getColliShape(){return collisionShapes;}
	public static void setgVertices(ByteBuffer newVer){gVertices=newVer;}
	
	public static int getCount(){return count;}
	public static void incrementCount() {count++;}
	public static long getTime(){
		final long endTime = System.currentTimeMillis();
		final long diff=endTime-start_time;
		return diff;
	}
	
	
	
	@Override
	public void clientMoveAndDisplay() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//float ms = getDeltaTimeMicroseconds();
		// step the simulation
		if (dynamicsWorld != null) {
			if(counter==0){	start_time = System.currentTimeMillis();}
			dynamicsWorld.stepSimulation((float)1/60); //step the world once 1/5 sec.
            InternalTickCallback cb=new Model4(dynamicsWorld, gl);//MyInternalTickCallback ();
			Object worldUserInfo=0;
			
			dynamicsWorld.setInternalTickCallback(cb, worldUserInfo);
			dynamicsWorld.debugDrawWorld();
		}
		counter++;
		renderme();
		if(count==1000)	{		
        	toTxtFile(positionList,1);	//toTxtFile(positionList,2);	toTxtFile(positionList,3);	toTxtFile(positionList,4);
		    String start="input=[";
		    for (int i=0;i<27;i++){ start+=(Model4.getInputDis()[i]+","); }    
		    start+="];";
		 System.out.println(start);
         }
	}

	
	@Override
	public void displayCallback() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		renderme();
		// optional but useful: debug drawing to detect problems
		if (dynamicsWorld != null) {
			dynamicsWorld.debugDrawWorld();
		}
		//glFlush();
		//glutSwapBuffers();
	}


	
	public void initPhysics() throws IOException {
		setCameraDistance(250f);
		// collision configuration contains default setup for memory, collision setup
		collisionConfiguration = new DefaultCollisionConfiguration();
		// use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		dispatcher = new CollisionDispatcher(collisionConfiguration);
		broadphase = new DbvtBroadphase();
		SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
		solver = sol;		
		// TODO: needed for SimpleDynamicsWorld
		//sol.setSolverMode(sol.getSolverMode() & ~SolverMode.SOLVER_CACHE_FRIENDLY.getMask());		
		dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		dynamicsWorld.setGravity(new Vector3f(0f, 0f, 20f));
        
		
		CollisionShape groundShape = new StaticPlaneShape(new Vector3f(0, 0, -1), 0);
		collisionShapes.add(groundShape);
		Transform groundTransform = new Transform();
		groundTransform.setIdentity();
		groundTransform.origin.set(0, 0, 0);
        float mass = 0f; 
		// using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		MotionState myMotionState = new DefaultMotionState(groundTransform);
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, groundShape, new Vector3f(0, 0, 0));
		RigidBody body = new RigidBody(rbInfo);
		dynamicsWorld.addRigidBody(body);
		
		
		//add in the triangle mesh for dish border
		ByteBuffer gVertices2;
		ByteBuffer gIndices2;
        gVertices2 = ByteBuffer.allocateDirect(dishPoints*4*3).order(ByteOrder.nativeOrder());	
        gVertices2.clear();
		 for (int i=0;i<dishPoints;i++){
			 gVertices2.putFloat((float)(dishRadius*Math.cos(2*Math.PI*i/dishPoints)));
			 gVertices2.putFloat((float)(dishRadius*Math.sin(2*Math.PI*i/dishPoints)));	
			 gVertices2.putFloat(-(float)i%2*dishHeight);}	 
		 gIndices2 = ByteBuffer.allocateDirect(numOfTriangles*3*4).order(ByteOrder.nativeOrder());		
		 gIndices2.clear();
		for (int i=0;i<numOfTriangles;i++){
				gIndices2.putInt((i+2)%dishPoints);
				gIndices2.putInt((i+1)%dishPoints);
				gIndices2.putInt((i)%dishPoints);
		}
		//gIndices.flip();
		int vertStride = 3 * 4;
		int indexStride = 3 * 4;
		TriangleIndexVertexArray indexVertexArrays = new TriangleIndexVertexArray(numOfTriangles,gIndices2,indexStride, dishPoints, gVertices2, vertStride);
		BvhTriangleMeshShape dishShape = new BvhTriangleMeshShape(indexVertexArrays, true);	
		//dishShape.setUserPointer(userPtr)//collisionShapes.add(dishShape);		
		Transform triTransform = new Transform();
		triTransform.setIdentity();
		triTransform.origin.set(0, 0, 0);
         float mass2 = 0f;
		DefaultMotionState myMotionState2 = new DefaultMotionState(triTransform);
		RigidBodyConstructionInfo rbInfo2 = new RigidBodyConstructionInfo(mass2, myMotionState2, dishShape, new Vector3f(0, 0, 0));
		dish = new RigidBody(rbInfo2);
		//dish.setFriction(0);
		dynamicsWorld.addRigidBody(dish);

		
		
		
        //add in soil at the bottom
		int totalTriangles = 2 * (NUM_VERTS_X - 1) * (NUM_VERTS_Y - 1);
		gVertices = ByteBuffer.allocateDirect(totalVerts * 3 * 4).order(ByteOrder.nativeOrder());
		gIndices = ByteBuffer.allocateDirect(totalTriangles * 3 * 4).order(ByteOrder.nativeOrder());
		int d=dishRadius;
		for (int i =0; i <NUM_VERTS_X ; i++) { 
			 for (int j = 0; j <NUM_VERTS_Y ; j++) {
				gVertices.putFloat(i*10-d-5);
				gVertices.putFloat(j*10-d-5);
				gVertices.putFloat(0);
				soilMeshHeight.add((float) 0);
			}
		}
		
		//for (int i=0;i<1000;i++){
			//System.out.println("gVertices.getFloat"+i+" is "+gVertices.getFloat(i));
		//}
		gIndices.clear();
		for (int i = 0; i < NUM_VERTS_X - 1; i++) {
			for (int j1 = 0; j1 < NUM_VERTS_Y - 1; j1++) {
				gIndices.putInt(j1 * NUM_VERTS_X + i);
				gIndices.putInt(j1 * NUM_VERTS_X + i + 1);
				gIndices.putInt((j1 + 1) * NUM_VERTS_X + i + 1);
				gIndices.putInt(j1 * NUM_VERTS_X + i);
				gIndices.putInt((j1 + 1) * NUM_VERTS_X + i + 1);
				gIndices.putInt((j1 + 1) * NUM_VERTS_X + i);
			}
		}
		gIndices.flip();
		soilPoints= new TriangleIndexVertexArray(totalTriangles,gIndices,indexStride,totalVerts, gVertices, vertStride);
		soil = new BvhTriangleMeshShape(soilPoints, true);	
		collisionShapes.add(soil);
		Transform triTransform2 = new Transform();
		triTransform2.setIdentity();
		triTransform2.origin.set(0, 0, 0);
		DefaultMotionState SoilMotionState = new DefaultMotionState(triTransform2);
		RigidBodyConstructionInfo SoilrbInfo = new RigidBodyConstructionInfo(0f, SoilMotionState, soil, new Vector3f(0, 0, 0));
		soilBody = new RigidBody(SoilrbInfo);

		dynamicsWorld.addRigidBody(soilBody);		
		

		
			
		//create numOfTermites capsules that align with the x axis.
		for (int i1=0; i1<numOfTermites;i1++){ 
			CapsuleShapeX terShape = new CapsuleShapeX(termiteRadius, termiteLen);
			collisionShapes.add(terShape);
			Transform capTransform = new Transform();
			capTransform.setIdentity();
			float mass3 = 5f;
			boolean isDynamic3 = (mass3 != 0f);
			Vector3f localInertia3 = new Vector3f(0, 0, 0);
			if (isDynamic3) {terShape.calculateLocalInertia(mass3, localInertia3);}
			
			//initialize the termite location at random. Add the position to positionList
			double radius = (Math.random()*(dishRadius-20)); //Math.random() returns a double value between 0.0 and 1.0 between 0 and the radius of the circle
			float angle1 = (float) (Math.random()*2*Math.PI); // between 0 and 360 (degrees)  
			float x =(float) (radius*Math.cos(angle1));
			float y = (float) (radius*Math.sin(angle1)); 
			capTransform.origin.set(x,y,termiteHeight);
            
			//set the beginning orientation to a random orientation
			float roll = 0, pitch = 0, yaw =(float) (Math.random()*2*Math.PI);
			float randomAngle=(float) (Math.random()*2*Math.PI);
			capTransform.basis.rotZ(randomAngle);
	      
			DefaultMotionState myMotionState3 = new DefaultMotionState(capTransform);
			RigidBodyConstructionInfo rbInfo3 = new RigidBodyConstructionInfo(mass3, myMotionState3, terShape, localInertia3);
			RigidBody body3 = new RigidBody(rbInfo3);
			//body3.setAngularFactor(new Vector3f(0,0, 0.0)); could not rotate around itself!
          // body3.setFriction(0);
			states[i1]=0;

			termites.add(body3);
			dynamicsWorld.addRigidBody(body3);
			Quat4f ori=new Quat4f();
			ori=body3.getOrientation(ori);
			float angle=BasicDemo2.getAngle(ori);
			float center_x=x;
			float center_y=y;
			float termiteHalfLen=(termiteLen/2);
			float head_x=(float) (center_x+termiteHalfLen*Math.cos(angle));
			float head_y=(float) (center_y+termiteHalfLen*Math.sin(angle));
			float tail_x=(float) (center_x-termiteHalfLen*Math.cos(angle));
			float tail_y=(float) (center_y-termiteHalfLen*Math.sin(angle));
			
			ArrayList<Float> posList=new ArrayList<Float>();
			posList.add(head_x);	posList.add(head_y);	posList.add(tail_x);	posList.add(tail_y);
			positionList.add(posList);

		}
		
		for (int i=0;i<=26;i++){
			
			int[] s=new int[4];
			for (int state=0;state<=3;state++){
				s[state]=0;
			}
			timeSpentInOneState.add(s);
		}
		clientResetScene();
	
	}
	
	
	
	
	public static float getHeight(BvhTriangleMeshShape mesh, int num_x,int num_y){
		//first point: x(0),y(4), y(8), first point
		//second point: x(12), y(16), z(20), seconf point
		//third point:x (24), y(28) , z(32)
		int point=num_x*NUM_VERTS_Y+num_y;
		int position=point*12+8;
		return gVertices.getFloat(position);
	}
	
	
	
	/**
	 * Output the center position and orientation of each termite to a txt file. (positionList and orientationList)
	 * The head and tail positions could be calculated from the position and orientation.
	 */
	public void toTxtFile(ArrayList<ArrayList<Float>> posList, int block_num){
		for(int i=0;i<posList.size();i++){
			ArrayList<Float> dataForOneTermite=posList.get(i);
			String content = "";
			for(int j=(block_num-1)*4000;j<block_num*4000;j++){
				Float point=dataForOneTermite.get(j);
				content=content+point.toString()+ " ";
			}
			FileOutputStream fop = null;
			File file;
			String filename = "D:\\Yixin\\model\\2\\Model2Block"+block_num+"Term"+(i+1)+".txt";
	 
			try {
				file = new File(filename);
				fop = new FileOutputStream(file);
	             if (!file.exists()) {file.createNewFile();}
	             // get the content in bytes
				byte[] contentInBytes = content.getBytes();
	 
				fop.write(contentInBytes);
				fop.flush();
				fop.close();
	 
			} catch (IOException e) {
				e.printStackTrace();
			} finally {
				try {
					if (fop != null) {
						fop.close();
					}
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
	}
	
	
	
	public static void main(String[] args) throws LWJGLException, IOException {
		BuildingDemo ccdDemo = new BuildingDemo(LWJGL.getGL());
		ccdDemo.initPhysics();
		ccdDemo.getDynamicsWorld().setDebugDrawer(new GLDebugDrawer(LWJGL.getGL()));
		LWJGL.main(args, 800, 600, "Termite", ccdDemo);
	}

}
