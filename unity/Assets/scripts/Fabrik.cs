using UnityEngine;
using System.Collections;


public class Fabrik : MonoBehaviour {
	
	public Transform target;
	public Transform ikChain;
	
	public class JointInfo
	{
		public readonly float distanceToParent_; // if 0, no parent
		public readonly float distanceToRoot_; // if 0, no parent
		public readonly JointInfo parent_;
		public readonly JointInfo[] children_;
		public readonly JointInfo[] effectors_;
		
		public JointInfo (Transform joint) : this(joint, null)
		{
			// NOTHING	
		}
		
		private JointInfo (Transform joint, JointInfo parent)
		{
			parent_ = parent;
			if(parent == null)
			{
				distanceToParent_ = 0;
				distanceToRoot_ = 0;
			}
			else
			{
				distanceToParent_ = Vector3.Distance (joint.parent.position, joint.position);
				distanceToRoot_ = parent.distanceToRoot_ + distanceToParent_;
			}
			int nbChildren = joint.childCount;
			children_ = new JointInfo[nbChildren];
			children_ = new JointInfo[nbChildren];
			if(joint.childCount == 0) // joint is effector
			{
				effectors_ = new JointInfo[1];
				effectors_[0] = this;
			}
			else
			{
				int nbEffectors_ = 0;
				for(int i=0; i < nbChildren; ++i)
				{
					JointInfo jInfo =new JointInfo(joint.GetChild(i), this);
					children_[i] = jInfo;
					nbEffectors_ += jInfo.effectors_.Length;
				}
				effectors_ = new JointInfo[nbEffectors_];
				int j = 0;
				foreach(JointInfo jInfo in children_)
				{
					foreach(JointInfo effector in jInfo.effectors_)
					{
						effectors_[j++] = effector;
					}
				}
			}
		}
	}
	
	public JointInfo jInfo;
		
	// Use this for initialization
	void Start ()
	{
		// NOTHING
		jInfo = new	JointInfo(ikChain);
	}
	
	// Update is called once per frame
	void Update ()
	{
		if (target != null)
		{
			FabrikStep();
		}
	}
	
	private void FabrikStep()
	{
		Vector3 rootPos = ikChain.position;
		// first going from down to up
		JointInfo effector = jInfo.effectors_[0];
		JointInfo currentJoint = jInfo;
		Transform currentTransform = ikChain;
		if(Vector3.Distance(rootPos, target.position)  > effector.distanceToRoot_)
		{
			IkOutOfRangeTarget(jInfo, ikChain);
		}
	}
	
	private void IkOutOfRangeTarget(JointInfo currentJoint, Transform ikChain)
	{
		//Find the distance ri between the target t and the joint
		float r = (target.position - ikChain.position).magnitude;
		for (int i = 0; i < currentJoint.children_.Length; ++i)
		{
			JointInfo son = currentJoint.children_[i];
			Transform chainSon = ikChain.GetChild(i);
			float delta = son.distanceToParent_ / r;
			Vector3 newPos = (1 - delta) *  ikChain.position + delta * target.position;
			MoveTransformToPosition(son, chainSon, newPos);
			IkOutOfRangeTarget(son, chainSon);
		}
	}
	
	private void MoveTransformToPosition(JointInfo currentJoint, Transform transform, Vector3 target)
	{
		if(currentJoint.parent_ != null) transform.position = target;
		//transform.position = target;
		//transform.LookAt(target);
	}
}
