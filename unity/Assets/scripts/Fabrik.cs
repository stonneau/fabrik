using UnityEngine;
using System.Collections;


public class Fabrik : MonoBehaviour {
	
	public Transform target;
	public Transform ikChain;
	
	public class JointInfo
	{
		public readonly float distanceToParent_; // if 0, no parent
		public readonly float distanceToRoot_; // if 0, no parent
		public readonly int id_; // if 0, no parent
		public readonly JointInfo parent_;
		public readonly JointInfo[] children_;
		public readonly JointInfo[] effectors_;
		
		public JointInfo (Transform joint, ref int id) : this(joint, ref id, null)
		{
			// NOTHING	
		}
		
		private JointInfo (Transform joint, ref int id, JointInfo parent)
		{
			id_ = id;
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
					++id;
					JointInfo jInfo = new JointInfo(joint.GetChild(i), ref id, this);
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
	
	private JointInfo jInfo;
	private Transform[] transforms;
	
	public float treshold;
		
	// Use this for initialization
	void Start ()
	{
		int id = 0; // id is updated with a depth-first iteration
		jInfo = new	JointInfo(ikChain, ref id);
		transforms = new Transform[id+1];
		id = 0;
		InitTransform(ikChain, ref id);
	}
	
	// Update is called once per frame
	void Update ()
	{
		if (target != null)
		{
			FabrikStep();
		}
	}
	
	private void InitTransform(Transform transform, ref int id)
	{
		transforms[id] = transform;
		for (int i=0; i < transform.GetChildCount(); ++i)
		{
			++ id;
			InitTransform(transform.GetChild(i), ref id);
		}
	}
	
	private void FabrikStep()
	{
		Vector3 rootPos = ikChain.position;
		// first going from down to up
		JointInfo effector = jInfo.effectors_[0]; // first approach : one effector only
		// target too far from tree
		if(Vector3.Distance(rootPos, target.position)  > effector.distanceToRoot_)
		{
			IkOutOfRangeTarget(jInfo);
		}
		// target reachable (in theory), make sure that not already reached
		else if((transforms[effector.id_].position - target.position).magnitude  > treshold)
		{
			// forward reaching first
			transforms[effector.id_].position = target.position;
			ForwardStep(effector);
			// then backward reaching 
			ikChain.position = rootPos;
			BackwardStep(jInfo);
		}
	}
	
	private void IkOutOfRangeTarget(JointInfo currentJoint)
	{
		Transform ikChain = transforms[currentJoint.id_];
		//Find the distance ri between the target t and the joint
		float r = (target.position - ikChain.position).magnitude;
		for (int i = 0; i < currentJoint.children_.Length; ++i)
		{
			JointInfo son = currentJoint.children_[i];
			Transform chainSon = ikChain.GetChild(i);
			float delta = son.distanceToParent_ / r;
			Vector3 newPos = (1 - delta) *  ikChain.position + delta * target.position;
			MoveTransformToPosition(son, chainSon, newPos);
			IkOutOfRangeTarget(son);
		}
	}
	
	private void ForwardStep(JointInfo currentJoint)
	{
		if(currentJoint.parent_ != null)
		{
			Transform transform = transforms[currentJoint.id_];
			Transform parentTransform = transforms[currentJoint.parent_.id_];
			float r = Vector3.Distance(transform.position, parentTransform.position);
			float delta = currentJoint.distanceToParent_ / r;
			Vector3 newPos = (1 - delta) *  transform.position + delta * parentTransform.position;
			MoveTransformToPosition(currentJoint.parent_, parentTransform, newPos);
			ForwardStep(currentJoint.parent_);
		}
	}
	
	private void BackwardStep(JointInfo currentJoint)
	{
		if(currentJoint.children_.Length != 0)
		{
			JointInfo sonJoint = currentJoint.children_[0];
			Transform transform = transforms[currentJoint.id_];
			Transform childTransform = transforms[sonJoint.id_];
			float r = Vector3.Distance(transform.position, childTransform.position);
			float delta = sonJoint.distanceToParent_ / r;
			Vector3 newPos = (1 - delta) * transform.position+ delta * childTransform.position;
			MoveTransformToPosition(sonJoint, childTransform, newPos);
			BackwardStep(sonJoint);
		}
	}
	
	private void MoveTransformToPosition(JointInfo currentJoint, Transform transform, Vector3 target)
	{
		if(currentJoint.parent_ != null) transform.position = target;
		//transform.position = target;
		//transform.LookAt(target);
	}
}
