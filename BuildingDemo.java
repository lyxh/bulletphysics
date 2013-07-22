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
	private static int numOfTermites=22;
	private static int[] states=new int[numOfTermites];
	private static ArrayList<int[]> timeSpentInOneState= new ArrayList<int[]>();
	private static float termiteRadius=5;
    private static float termiteLen=26;
    private float termiteHeight=-6;
    private static ArrayList<ArrayList<Vector3f>> positionList= new ArrayList<ArrayList<Vector3f>>();
    private static ArrayList<ArrayList<Quat4f>> orientationList= new ArrayList<ArrayList<Quat4f>>();
    private static float time=0; 
    
 	private static int totalDataNum=11989;
 	//result(caseCount,3,caseNum) in matlab
 	public static ArrayList<float[][]> trackingData=new ArrayList<float[][]>();
	//private String caseDataPath="//mit//liyixin//Desktop//SUMMER//model//Case_";
	//private String caseCountPath="//mit//liyixin//Desktop//SUMMER//model//Case_Count_Model_2.txt";
 	private String caseDataPath="D:\\Yixin\\model\\Case_";
 	private String caseCountPath="D:\\Yixin\\model\\Case_Count_Model_2.txt";
 	
 	private static int[] caseCount=new int[27];
	private static ArrayList<Vector3f> force=new ArrayList<Vector3f>();
	private static int counter=0;
 	
	public BuildingDemo(IGL gl) {super(gl);}  
	public static ObjectArrayList<RigidBody> getTermites(){	return termites;}	
	public static ArrayList<ArrayList<Vector3f>> getPositionList(){return positionList;}	
	public static ArrayList<ArrayList<Quat4f>> getOrientationList(){return orientationList; }	
	public static BvhTriangleMeshShape getSoilMesh(){	return soil;}
	public static void setSoilMesh(BvhTriangleMeshShape newSoil){	soil=newSoil;}
	
	public static RigidBody getSoil(){return soilBody;}
	public static void setSoil( RigidBody newSoilBody){soilBody=newSoilBody;}
	
	public static float getTermiteLen(){return termiteLen;}
	public static float getTermiteRad(){return termiteRadius;}
	public static float getDishRadius(){return dishRadius;}
	public static int getTermiteCount(){return numOfTermites;}
	public static int[] getStates(){return states;}
	public static ArrayList<float[][]>  getData(){return trackingData;}
	public static int[] getCaseCount() {return caseCount;}
	public static int getCounter() {return counter;}
	public static  ArrayList<Vector3f>  getForce() {return force;}
	public static void setForce(Vector3f newforce, int termite) {force.set(termite,newforce);}
	public static int getState(int i){return states[i];}
	public static void setState(int newstate, int i){states[i]=newstate;}
	public static int[] getTimeInState(int termite){return timeSpentInOneState.get(termite);}
	public static void setTimeInState(int termite, int stateNum, int newVal){timeSpentInOneState.get(termite)[stateNum]=newVal;}
	public static void clearTimeInState(int termite){
		for (int state=0; state<=3;state++){timeSpentInOneState.get(termite)[state]=0;}
	}

	static ObjectArrayList<CollisionShape> getColliShape(){return collisionShapes;}
	
	public static void setgVertices(ByteBuffer newVer){gVertices=newVer;}
	@Override
	public void clientMoveAndDisplay() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//float ms = getDeltaTimeMicroseconds();
		// step the simulation
		if (dynamicsWorld != null) {
			//TODO: set the time to be correct
			dynamicsWorld.stepSimulation((float)time,1); //step the world once 1/5 sec.
		
            InternalTickCallback cb=new Model3(dynamicsWorld, gl);//MyInternalTickCallback ();
			Object worldUserInfo=0;
			
			dynamicsWorld.setInternalTickCallback(cb, worldUserInfo);
			dynamicsWorld.debugDrawWorld();
		}
		time+=0.2;//getDeltaTimeMicroseconds()/1000000;
		counter++;
		renderme();
		
		if (time>=1000){
			for (int i=0;i<27;i++){ 
				int j=i+1;
				if( i==0){System.out.println("start");}
				System.out.println(Model3.getInputDis()[i]+",");
				}
		}
		
		//glFlush();
		//glutSwapBuffers();
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
		Object imageFilename;
		//TODO: Set the texture of termites,floor, wall
		//loadBMP(imageFilename);   // Load BMP image
		//Object textureIDs;
		//glGenTextures(1, textureIDs); // Generate 1 texture ID
		
		// collision configuration contains default setup for memory, collision setup
		collisionConfiguration = new DefaultCollisionConfiguration();
		// use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		dispatcher = new CollisionDispatcher(collisionConfiguration);
		broadphase = new DbvtBroadphase();
		// the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
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
			float angle = (float) (Math.random()*2*Math.PI); // between 0 and 360 (degrees)  
			float x =(float) (radius*Math.cos(angle));
			float y = (float) (radius*Math.sin(angle)); 
			capTransform.origin.set(x,y,termiteHeight);
            
			//set the beginning orientation to a random orientation
			float roll = 0, pitch = 0, yaw =(float) (Math.random()*2*Math.PI);
			float randomAngle=(float) (Math.random()*2*Math.PI);
			capTransform.basis.rotZ(randomAngle);
			
			ArrayList<Vector3f> individualList=new ArrayList<Vector3f>();
			individualList.add(new Vector3f(x,y,termiteHeight));
			positionList.add(individualList);
	      
			DefaultMotionState myMotionState3 = new DefaultMotionState(capTransform);
			RigidBodyConstructionInfo rbInfo3 = new RigidBodyConstructionInfo(mass3, myMotionState3, terShape, localInertia3);
			RigidBody body3 = new RigidBody(rbInfo3);
			body3.setSleepingThresholds((float)0.0, (float)0.0);
			//body3.setAngularFactor(new Vector3f(0,0, 0.0)); could not rotate around itself!
          // body3.setFriction(0);
			states[i1]=0;

			termites.add(body3);
			dynamicsWorld.addRigidBody(body3);
			
			// Add initial orientation to the orientationList
			Quat4f ori=new Quat4f();
			ori=body3.getOrientation(ori);
			ArrayList<Quat4f> individualOriList=new ArrayList<Quat4f>();
			individualOriList.add(ori);
			orientationList.add(individualOriList);
		}
		
		this.caseCount=readCaseCount(caseCountPath);
		for (int i=0;i<=26;i++){
			float[][] data=readDistributionData(caseDataPath, i);
			this.trackingData.add(data);
			force.add(new Vector3f(0,0,0));
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
	public void toTxtFile(){
		
	}
	
	   /**
	 * @return 
	 * @throws IOException 
	    * 
	    */
	public static float[][] readDistributionData(String filePath,int i) throws IOException {
		int c=caseCount[i];
			float[][] result= new float[c][3];
			int j=1+i;
			filePath+=j;
			filePath+=".txt";
			byte[] buffer = new byte[(int) new File(filePath).length()];
			    BufferedInputStream f = null;
			    try {f = new BufferedInputStream(new FileInputStream(filePath));
			        f.read(buffer);
			        if (f != null) try { f.close(); } catch (IOException ignored) { }} 
		        catch (IOException ignored) { System.out.println("File not found or invalid path.");}			    
			
			   String[] lines=new String(buffer).split("\\s+");
			   for  (int  count=0;count<c*3;count++){	             
			    		  BigDecimal number = new BigDecimal(lines[count]);
				    	  String numWithNoExponents = number.toPlainString();
				    	  float num=Float.valueOf(numWithNoExponents);
				    	  int mod=count/c;
				    	  int div=count%c;		
				    	 //System.out.println("mod: " +mod + "; div: "+div); 
				          result[div][mod]= num;			         
				          //System.out.println("result["+div +"]["+mod+"]="+result[div][mod]);				   
			   }
			return result;
		}
	
	
	
	public static int[] readCaseCount(String filePath) {
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
	
	
	public static void main(String[] args) throws LWJGLException, IOException {
		BuildingDemo ccdDemo = new BuildingDemo(LWJGL.getGL());
		ccdDemo.initPhysics();
		ccdDemo.getDynamicsWorld().setDebugDrawer(new GLDebugDrawer(LWJGL.getGL()));
		LWJGL.main(args, 800, 600, "Termite", ccdDemo);
		ccdDemo.toTxtFile();
	}




	
	
}
