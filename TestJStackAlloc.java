package com.bulletphysics;

import javax.vecmath.Vector3f;

import cz.advel.stack.Stack;

public class TestJStackAlloc
{

	/**
	 * @param args
	 */
	public static void main(String[] args)
	{
		Vector3f tmp = Stack.alloc(Vector3f.class);
		tmp.set(1,2,3);
		System.out.println(tmp.x);
		System.out.println(tmp.y);
		System.out.println(tmp.z);
		System.out.println(tmp.x);
		System.out.println(tmp.z);
		System.out.println(tmp.y);

	}

}
