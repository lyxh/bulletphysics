package com.bulletphysics;

import java.util.ArrayList;

import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;
import javax.vecmath.Vector4f;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.InternalTickCallback;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.BasicDemo;

public class CopyOfMyInternalTickCallback extends InternalTickCallback{
	
	
	public void internalTick(DynamicsWorld dynamicsWorld, float timeStep) {		
		ObjectArrayList<RigidBody> termites= BasicDemo.getTermites();
		ArrayList<ArrayList<Vector3f>> posList=BasicDemo.getPositionList();
		ArrayList<ArrayList<Quat4f>> oriList=BasicDemo.getOrientationList();
	    //get the position and orientation of each termite
		for (int j=0; j<termites.size();j++){	
			Vector3f position= new Vector3f(0,0,0);
		   	// Position right now is in world coordinate system.
			position=termites.get(j).getCenterOfMassPosition(position);
			Quat4f orientation=new Quat4f();
			orientation=termites.get(j).getOrientation(orientation);
			posList.get(j).add(position);
			oriList.get(j).add(orientation);
			//System.out.println(orientation);
		}
		
		
		// for each frame, set the velocity:just try to move straight ahead
		for (int r=0; r<termites.size(); r++) {
			RigidBody body= termites.get(r);			
			Vector3f localforce= new Vector3f(2,0,0);
			Transform t=new Transform();
			t=body.getMotionState().getWorldTransform(t);
			//transform the force from local coordinates to global coordinates.
			Vector3f globalForce=new Vector3f(0,0,0);
			globalForce.x=localforce.dot(new Vector3f(t.basis.m00, t.basis.m10, t.basis.m20));
			globalForce.y=localforce.dot(new Vector3f(t.basis.m01, t.basis.m11, t.basis.m21));
			globalForce.z=1;//localforce.dot(new Vector3f(t.basis.m02, t.basis.m12, t.basis.m22));
			//System.out.println(globalForce);
			//System.out.println(t.basis);
			body.setLinearVelocity(globalForce);
			//clear all turns
			body.setAngularVelocity(new Vector3f(0,0,0));
			//Vector3f angularVelocity= new Vector3f((int)Math.random()*10,(int)Math.random()*20,(int)Math.random()*10);
			// setAngularVelocity unit: rad/s 
			//TODO: Classify the input and apply the right force.
			
		}
		

		// if there is a collision, turn left/right with half of the probability
		//Assume world->stepSimulation or world->performDiscreteCollisionDetection has been called 
		int numManifolds = dynamicsWorld.getDispatcher().getNumManifolds();
		for (int i=0;i<numManifolds;i++)
		{
			PersistentManifold contactManifold =  dynamicsWorld.getDispatcher().getManifoldByIndexInternal(i);
			CollisionObject obA = (CollisionObject)(contactManifold.getBody0());
			CollisionObject obB = (CollisionObject)(contactManifold.getBody1());
			RigidBody bodyA=(RigidBody) (contactManifold.getBody0());
			RigidBody bodyB=(RigidBody) (contactManifold.getBody1());
			int numContacts = contactManifold.getNumContacts();
			for (int j=0;j<numContacts;j++){	
				ManifoldPoint pt = contactManifold.getContactPoint(j);
				if (pt.getDistance()<0.f){  //collision!
					//TODO: only collision with the termites and the wall. Might not need this for the model.
					/*
					Vector3f forceA= new Vector3f(0,0,2);
					Transform tA=new Transform();
					tA=bodyA.getMotionState().getWorldTransform(tA);
					Vector3f globalForceA=new Vector3f(0,0,0);
					globalForceA.x=forceA.dot(new Vector3f(tA.basis.m00, tA.basis.m10, tA.basis.m20));
					//globalForce.y=localforce.dot(new Vector3f(t.basis.m01, t.basis.m11, t.basis.m21));
					globalForceA.z=forceA.dot(new Vector3f(tA.basis.m02, tA.basis.m12, tA.basis.m22));
					bodyA.setLinearVelocity(globalForceA);
					
					Vector3f forceB= new Vector3f(0,0,-2);
					Transform tB=new Transform();
					tB=bodyB.getMotionState().getWorldTransform(tB);
					Vector3f globalForceB=new Vector3f(0,0,0);
					globalForceB.x=forceB.dot(new Vector3f(tB.basis.m00, tB.basis.m10, tB.basis.m20));
					//globalForce.y=localforce.dot(new Vector3f(tB.basis.m01, tB.basis.m11, tB.basis.m21));
					globalForceB.z=forceB.dot(new Vector3f(tB.basis.m02, tB.basis.m12, tB.basis.m22));
					bodyB.setLinearVelocity(globalForceB);
                    */
					//Vector3f ptA = pt.getPositionWorldOnA(ptA);
					//Vector3f ptB = pt.getPositionWorldOnB(ptB);
				}
			}
		
		}
	}



}
