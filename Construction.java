package com.bulletphysics;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.collision.shapes.TriangleIndexVertexArray;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;

public class Construction {
	private static int dishRadius=(int) BuildingDemo.getDishRadius();
	private static int NUM_VERTS_X = (1+dishRadius/10)*2;
	private static int NUM_VERTS_Y = NUM_VERTS_X ;
	private static int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
	private static float height=0; //positive height moves the soil lower
	private static int d=dishRadius;
	private static int range=20;

    public static void dig(RigidBody termite, Vector3f position, float position_x, float position_y, DynamicsWorld dynamicsWorld){
    	//get the termite position
    	int x=Math.round((position_x+205)/(float)10);
		int y=Math.round((position_y+205)/(float)10);
        //add in soil at the bottom
		int totalTriangles = 2 * (NUM_VERTS_X - 1) * (NUM_VERTS_Y - 1);
		ByteBuffer gVertices = ByteBuffer.allocateDirect(totalVerts * 3 * 4).order(ByteOrder.nativeOrder());
		ByteBuffer gIndices = ByteBuffer.allocateDirect(totalTriangles * 3 * 4).order(ByteOrder.nativeOrder());
		int d=dishRadius;
		for (int i =0; i <NUM_VERTS_X ; i++) { 
			 for (int j = 0; j <NUM_VERTS_Y ; j++) {
				 height=BuildingDemo.getHeight(BuildingDemo.getSoilMesh(),i,j);		
				 if(i==x && j==y){height=10;}
				gVertices.putFloat(i*10-d-5);
				gVertices.putFloat(j*10-d-5);
				gVertices.putFloat(height);
			}
		}
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
		TriangleIndexVertexArray soilPoints= new TriangleIndexVertexArray(totalTriangles,gIndices,12,totalVerts, gVertices, 12);
		BvhTriangleMeshShape soil = new BvhTriangleMeshShape(soilPoints, true);	
		BuildingDemo.getColliShape().add(soil);
		Transform triTransform2 = new Transform();
		triTransform2.setIdentity();
		triTransform2.origin.set(0, 0, 0);
		DefaultMotionState SoilMotionState = new DefaultMotionState(triTransform2);
		RigidBodyConstructionInfo SoilrbInfo = new RigidBodyConstructionInfo(0f, SoilMotionState, soil, new Vector3f(0, 0, 0));
		RigidBody soilBody = new RigidBody(SoilrbInfo);
		//set the new vertices
		BuildingDemo.setgVertices(gVertices);
		
		//remove the old shape, change the shape, and add it to collisionShapes
		BuildingDemo.getColliShape().remove(BuildingDemo.getSoilMesh());
		BuildingDemo.setSoilMesh(soil);
		BuildingDemo.getColliShape().add(BuildingDemo.getSoilMesh());
		
		//remove the old body, change the body and add it to the world
		dynamicsWorld.removeRigidBody(BuildingDemo.getSoil());
		BuildingDemo.setSoil(soilBody);
		dynamicsWorld.addRigidBody(soilBody);	
    }  
   
    
    public static void deposit(RigidBody termite, float position_x, float position_y,DynamicsWorld dynamicsWorld){
    	//get the termite position
    	int x=Math.round((position_x+205)/(float)10);
		int y=Math.round((position_y+205)/(float)10);
        //add in soil at the bottom
		int totalTriangles = 2 * (NUM_VERTS_X - 1) * (NUM_VERTS_Y - 1);
		ByteBuffer gVertices = ByteBuffer.allocateDirect(totalVerts * 3 * 4).order(ByteOrder.nativeOrder());
		ByteBuffer gIndices = ByteBuffer.allocateDirect(totalTriangles * 3 * 4).order(ByteOrder.nativeOrder());

		for (int i =0; i <NUM_VERTS_X ; i++) { 
			 for (int j = 0; j <NUM_VERTS_Y ; j++) {
				 height=BuildingDemo.getHeight(BuildingDemo.getSoilMesh(),i,j);
				 if (i==x && j==y){height=-10;}
				gVertices.putFloat(i*10-d-5);
				gVertices.putFloat(j*10-d-5);
				gVertices.putFloat(height);
			}
		}
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
		TriangleIndexVertexArray soilPoints= new TriangleIndexVertexArray(totalTriangles,gIndices,12,totalVerts, gVertices, 12);
		BvhTriangleMeshShape soil = new BvhTriangleMeshShape(soilPoints, true);	
		BuildingDemo.getColliShape().add(soil);
		Transform triTransform2 = new Transform();
		triTransform2.setIdentity();
		triTransform2.origin.set(0, 0, 0);
		DefaultMotionState SoilMotionState = new DefaultMotionState(triTransform2);
		RigidBodyConstructionInfo SoilrbInfo = new RigidBodyConstructionInfo(0f, SoilMotionState, soil, new Vector3f(0, 0, 0));
		RigidBody soilBody = new RigidBody(SoilrbInfo);
		//set the new vertices
		BuildingDemo.setgVertices(gVertices);
		
		//remove the old shape, change the shape, and add it to collisionShapes
		BuildingDemo.getColliShape().remove(BuildingDemo.getSoilMesh());
		BuildingDemo.setSoilMesh(soil);
		BuildingDemo.getColliShape().add(BuildingDemo.getSoilMesh());
		
		//remove the old body, change the body and add it to the world
		dynamicsWorld.removeRigidBody(BuildingDemo.getSoil());
		BuildingDemo.setSoil(soilBody);
		dynamicsWorld.addRigidBody(soilBody);	
		//System.out.println("Deposit called");
    }
    
    public static int[] pointToXY(int point_num){
    	 int[] result=new int[2];
    	 int x_num=point_num/NUM_VERTS_Y;
         int y_num=point_num%NUM_VERTS_Y;
         result[1]=x_num*10-d-5;
         result[2]=y_num*10-d-5;
         return result;
    }



	
	/**
	 * Test if a already exisiting digging site is in the sensing range of the termite
	 * @param head_x
	 * @param head_y
	 * @param tail_x
	 * @param tail_y
	 * @return
	 */
	public static boolean nearDig(float head_x, float head_y) {
		boolean result=false;
		// loop through every point that has a positive height in the soil map, 
		for (int i =0; i <NUM_VERTS_X ; i++) { 
			 for (int j = 0; j <NUM_VERTS_Y ; j++) {
				 height=BuildingDemo.getHeight(BuildingDemo.getSoilMesh(),i,j);
			     if(height>0){
					int x=i*10-d-5;
					int y=j*10-d-5;
					if( (Math.sqrt(x-head_x)+Math.sqrt(y-head_y))< range*range){ result=true;break;}	//test if the distance between point and termite head is less than sensing range
				}
			}
		}
		return result;
	}
	
	public static boolean nearDep(float head_x, float head_y) {
		boolean result=false;
		// loop through every point that has a positive height in the soil map, 
		for (int i =0; i <NUM_VERTS_X ; i++) { 
			 for (int j = 0; j <NUM_VERTS_Y ; j++) {
				 height=BuildingDemo.getHeight(BuildingDemo.getSoilMesh(),i,j);
			     if(height<0){
					int x=i*10-d-5;
					int y=j*10-d-5;
					if( (Math.sqrt(x-head_x)+Math.sqrt(y-head_y))< range*range){ result=true;break;}	//test if the distance between point and termite head is less than sensing range
				}
			}
		}
		return result;
	}
}
