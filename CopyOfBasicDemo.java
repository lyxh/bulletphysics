/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.collision.shapes.CapsuleShape;
import com.bulletphysics.collision.shapes.CapsuleShapeX;
import com.bulletphysics.collision.shapes.CapsuleShapeZ;
import com.bulletphysics.collision.shapes.CollisionShape;
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
import org.lwjgl.util.vector.Quaternion;

import static com.bulletphysics.demos.opengl.IGL.*;

/**
 */
public class CopyOfBasicDemo extends DemoApplication {
	// keep the collision shapes, for deletion/cleanup
	//private static DynamicsWorld dynamicsWorld;
	private ObjectArrayList<CollisionShape> collisionShapes = new ObjectArrayList<CollisionShape>();
	//private BvhTriangleMeshShape soil;
	private int soilHeight=0;
	private RigidBody dish;
	private BroadphaseInterface broadphase;
	private CollisionDispatcher dispatcher;
	private ConstraintSolver solver;
	private DefaultCollisionConfiguration collisionConfiguration;
    private static ObjectArrayList<RigidBody> termites= new ObjectArrayList<RigidBody>();
    private static ObjectArrayList<RigidBody> soilBlocks= new ObjectArrayList<RigidBody>();    
	private int dishRadius=20;
	private int dishPoints=200;
	private int numOfTriangles=dishPoints;
	private int dishHeight=5;
	private int numOfTermites=22;
	private float termiteRadius=1;
    private float termiteLen=4;
    private float soilSideLen=(float)1;
    private float termiteHeight=2;
	private RigidBody soilBody;
    private static ArrayList<ArrayList<Vector3f>> positionList= new ArrayList<ArrayList<Vector3f>>();
    private static ArrayList<ArrayList<Quat4f>> orientationList= new ArrayList<ArrayList<Quat4f>>();
    private static float time=0;
	public CopyOfBasicDemo(IGL gl) {super(gl);}
  
	public static ObjectArrayList<RigidBody> getTermites(){	return termites;}
	
	public static ArrayList<ArrayList<Vector3f>> getPositionList(){return positionList;}
	
	public static ArrayList<ArrayList<Quat4f>> getOrientationList(){return orientationList; }
	
	@Override
	public void clientMoveAndDisplay() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//float ms = getDeltaTimeMicroseconds();
		// step the simulation
		if (dynamicsWorld != null) {
			//TODO: set the time to be correct
			dynamicsWorld.stepSimulation((float)time,5); //step the world once 1/5 sec.
			// optional but useful: debug drawing
            InternalTickCallback cb=new MyInternalTickCallback();
			Object worldUserInfo=0;
			//dynamicsWorld.setInternalTickCallback(cb, worldUserInfo);
			dynamicsWorld.debugDrawWorld();
		}
		time+=0.2;//getDeltaTimeMicroseconds()/1000000;//
		renderme();
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

	public void initPhysics() {
		setCameraDistance(50f);
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
		dynamicsWorld.setGravity(new Vector3f(0f, -10f, 0f));

		
		CollisionShape groundShape = new StaticPlaneShape(new Vector3f(0, 1, 0), 0);
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
		
		
		//add in the triangle mesh
		ByteBuffer gVertices;
		ByteBuffer gIndices;
        gVertices = ByteBuffer.allocateDirect(dishPoints*4*3).order(ByteOrder.nativeOrder());	
        gVertices.clear();
		 for (int i=0;i<dishPoints;i++){
			 gVertices.putFloat((float)(dishRadius*Math.cos(2*Math.PI*i/dishPoints)));
			 gVertices.putFloat((float)i%2*dishHeight);
			 gVertices.putFloat((float)(dishRadius*Math.sin(2*Math.PI*i/dishPoints)));	}	 
		 gIndices = ByteBuffer.allocateDirect(numOfTriangles*3*4).order(ByteOrder.nativeOrder());		
		 gIndices.clear();
		for (int i=0;i<numOfTriangles;i++){
				gIndices.putInt((i+2)%dishPoints);
				gIndices.putInt((i+1)%dishPoints);
				gIndices.putInt((i)%dishPoints);
		}
		//gIndices.flip();
		int vertStride = 3 * 4;
		int indexStride = 3 * 4;
		TriangleIndexVertexArray indexVertexArrays = new TriangleIndexVertexArray(numOfTriangles,gIndices,indexStride, dishPoints, gVertices, vertStride);
		BvhTriangleMeshShape dishShape = new BvhTriangleMeshShape(indexVertexArrays, true);	
		collisionShapes.add(dishShape);		
		Transform triTransform = new Transform();
		triTransform.setIdentity();
		triTransform.origin.set(0, 0, 0);
         float mass2 = 0f;
		DefaultMotionState myMotionState2 = new DefaultMotionState(triTransform);
		RigidBodyConstructionInfo rbInfo2 = new RigidBodyConstructionInfo(mass2, myMotionState2, dishShape, new Vector3f(0, 0, 0));
		dish = new RigidBody(rbInfo2);
		dynamicsWorld.addRigidBody(dish);
		
		
		
		ByteBuffer gVertices2;
		ByteBuffer gIndices2;
		int sideLen=3;
		int dishPoints2=9;
		vertStride = 3 * 4;
		indexStride = 3 * 4;
		int numOfTriangles2=8;
        gVertices2 = ByteBuffer.allocateDirect(dishPoints2*4*3).order(ByteOrder.nativeOrder());	 
        gVertices2.clear();   
		for (int x=-1; x<=1;x+=1){
				for (int y=-1; y<=1;y+=1){
						 gVertices2.putInt(x);
						 gVertices2.putInt(1);
						 gVertices2.putInt(y);
				}
		}
		 gIndices2 = ByteBuffer.allocateDirect(numOfTriangles2*3*4).order(ByteOrder.nativeOrder());		
		 gIndices2.clear();
		 for (int x=0; x<2;x=x+1){
			 for (int y=0; y<2;y=y+1){
				 //one triangle
				gIndices2.putInt(x*3+y);
				gIndices2.putInt (x*3+y+1);
				gIndices2.putInt((x+1)*3+y);
				//another one

				gIndices2.putInt ((x+1)*3+y);
				gIndices2.putInt((x+1)*3+y+1);
				gIndices2.putInt(x*3+y+1);
			 }
		}
		
		 /*
		ByteBuffer gVertices2;
		ByteBuffer gIndices2;
		int sideLen=2*dishRadius+1;
		int dishPoints2=sideLen*sideLen;
		vertStride = 3 * 4;
		indexStride = 3 * 4;
		int numOfTriangles2=2*(sideLen-1)*(sideLen-1);
        gVertices2 = ByteBuffer.allocateDirect(dishPoints2*4*3).order(ByteOrder.nativeOrder());	 
        gVertices2.clear();
		//fill in all the soil.The trimesh is a square(hard to make it a circle?)
       
		for (int x=-dishRadius; x<=dishRadius;x=x+1){
				for (int y=-dishRadius; y<=dishRadius;y=y+1){
						 gVertices2.putInt(x);
						 gVertices2.putInt(soilHeight);
						 gVertices2.putInt(y);
				}
		}
		gVertices2.flip();
		 gIndices2 = ByteBuffer.allocateDirect(numOfTriangles2*3*4).order(ByteOrder.nativeOrder());		
		 gIndices2.clear();
		 for (int x=0; x<sideLen-1;x=x+1){
			 for (int y=0; y<sideLen-1;y=y+1){
				 //one triangle
				gIndices2.putInt(x);
				gIndices2.putInt (y);
				gIndices2.putInt(x+1);
				gIndices2.putInt(y);
				gIndices2.putInt (y+1);
				gIndices2.putInt(x);
				//gIndices2.putInt(x*sideLen+y);
				//gIndices2.putInt (x*sideLen+y+1);
				//gIndices2.putInt((x+1)*sideLen+y);
				//another triangle in the same square
				//gIndices2.putInt(x*sideLen+y+1);
				//gIndices2.putInt((x+1)*sideLen+y);
				//gIndices2.putInt((x+1)*sideLen+y+1);
			 }
		}
		*/
		    gIndices2.flip();
		    TriangleIndexVertexArray indexVertexArrays2 = new TriangleIndexVertexArray(numOfTriangles2,gIndices2,indexStride, dishPoints2, gVertices2, vertStride);
		    BvhTriangleMeshShape soil = new BvhTriangleMeshShape(indexVertexArrays2, true);	
			collisionShapes.add(soil);
			
			Transform triTransform2 = new Transform();
			triTransform2.setIdentity();
			triTransform2.origin.set(0, 0, 0);
			DefaultMotionState SoilMotionState = new DefaultMotionState(triTransform2);
			RigidBodyConstructionInfo SoilrbInfo = new RigidBodyConstructionInfo(0f, SoilMotionState, soil, new Vector3f(0, 0, 0));
			soilBody = new RigidBody(SoilrbInfo);

			dynamicsWorld.addRigidBody(soilBody);	

	
			
		//create numOfTermites capsules that align with the x axis.
		for (int i=0; i<numOfTermites;i++){ 
			CapsuleShapeX terShape = new CapsuleShapeX(termiteRadius, termiteLen);
			collisionShapes.add(terShape);
			Transform capTransform = new Transform();
			capTransform.setIdentity();
			float mass3 = 2f;
			boolean isDynamic3 = (mass3 != 0f);
			Vector3f localInertia3 = new Vector3f(0, 0, 0);
			if (isDynamic3) {terShape.calculateLocalInertia(mass3, localInertia3);}
			
			//initialize the termite location at random. Add the position to positionList
			double radius = (Math.random()*(dishRadius-4)); //Math.random() returns a double value between 0.0 and 1.0 between 0 and the radius of the circle
			double angle = Math.random()*(double)360; // between 0 and 360 (degrees)  
			float x =(float) (radius*Math.cos(angle));
			float y = (float) (radius*Math.sin(angle)); 
			capTransform.origin.set(x,termiteRadius+soilHeight,y);
	
			ArrayList<Vector3f> individualList=new ArrayList<Vector3f>();
			individualList.add(new Vector3f(x,termiteRadius+soilHeight,y));
			positionList.add(individualList);
	      
			DefaultMotionState myMotionState3 = new DefaultMotionState(capTransform);
			RigidBodyConstructionInfo rbInfo3 = new RigidBodyConstructionInfo(mass3, myMotionState3, terShape, localInertia3);
			RigidBody body3 = new RigidBody(rbInfo3);
			/*double orientation= Math.random()*(double)360;
			Transform tr = new Transform();
			tr.setIdentity();
			Quat4f quat=new Quat4f(10,10,10,10); //or quat.setEulerZYX depending on the ordering you want
			tr.setRotation(quat);
            body3.setCenterOfMassTransform(tr);*/
			//TODO:set the begin parameters. Set it to random orientation
			//Vector3f velocity= new Vector3f(0,0,2);
		   // body3.setLinearVelocity(velocity);
			//TODO: does not rotate the termites at the beginning.
			Vector3f angularVelocity= new Vector3f((int)Math.random()*8000,0,(int)Math.random()*10000);
			body3.setAngularVelocity(angularVelocity);
			termites.add(body3);
			dynamicsWorld.addRigidBody(body3);
		
			
			// Add initial orientation to the orientationList
			Quat4f ori=new Quat4f();
			ori=body3.getOrientation(ori);
			ArrayList<Quat4f> individualOriList=new ArrayList<Quat4f>();
			individualOriList.add(ori);
			orientationList.add(individualOriList);
		}
		clientResetScene();
		System.out.println("Init finished.");
	}
	
	/**
	 * Output the center position and orientation of each termite to a txt file. (positionList and orientationList)
	 * The head and tail positions could be calculated from the position and orientation.
	 */
	public void toTxtFile(){
		
	}
	
	public static void main(String[] args) throws LWJGLException {
		CopyOfBasicDemo ccdDemo = new CopyOfBasicDemo(LWJGL.getGL());
		ccdDemo.initPhysics();
		ccdDemo.getDynamicsWorld().setDebugDrawer(new GLDebugDrawer(LWJGL.getGL()));
		LWJGL.main(args, 800, 600, "Termite", ccdDemo);
		ccdDemo.toTxtFile();
	}
	
}
