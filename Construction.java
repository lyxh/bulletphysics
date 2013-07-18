package com.bulletphysics;

import javax.vecmath.Vector3f;

import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;

public class Construction {
	public Construction(){}
    public static void Construction(RigidBody termite, DynamicsWorld dynamicsWorld){
    	//get the termite position
		Vector3f position= new Vector3f(0,0,0);
		position=termite.getCenterOfMassPosition(position);
    	//decrease the soil height at that point through the trianglemesh
		BvhTriangleMeshShape soil= BasicDemo.getSoilMesh();
		Vector3f out = null;
		out=soil.localGetSupportingVertexWithoutMargin(position, out);
		out.y-=1;
		//TODO:find a way to change that specific point!
    }  
   
    
    public static void deposit(RigidBody termite, DynamicsWorld dynamicsWorld){
    	//get the termite position
		Vector3f position= new Vector3f(0,0,0);
		position=termite.getCenterOfMassPosition(position);
    	//decrease the soil height at that point through the trianglemesh
		BvhTriangleMeshShape soil= BasicDemo.getSoilMesh();
		Vector3f out = null;
		out=soil.localGetSupportingVertexWithoutMargin(position, out);
		out.y+=1;
    	
    }
}
