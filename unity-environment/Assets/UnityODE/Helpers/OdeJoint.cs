using System;
using UnityEngine;

public class OdeJointDefinition
{
    public enum Type
    {
        BallAndSocket,
        Hinge,
        Slider,
        Universal,
        Hinge2,
        PrismaticAndRotoide,
        PrismaticUniversal,
        Piston,
        Fixed,
        AngularMotor,
        LinearMotor, 
        Plane2D
    }
    internal static void CalculateDefaultMotorCfmErp(float timeStep, out float cfm, out float erp)
    {
        float kp = 10000.0f; //spring constant
        float kd = 0.5f; //spring damping

        erp = timeStep * kp / (timeStep * kp + kd);
        cfm = 1.0f / (timeStep * kp + kd);
    }

    #region Ball And Socket

    public static OdeJointDefinition CreateBallAndSocket(Vector3? loStop, Vector3? hiStop)
    {
        OdeJointDefinition def = new OdeJointDefinition();
        def.type = Type.BallAndSocket;

        if (loStop.HasValue)
        {
            Vector3 v = loStop.Value;
            def.lostop = v.x;
            def.lostop2 = v.y;
            def.lostop3 = v.z;
        }
        if (hiStop.HasValue)
        {
            Vector3 v = hiStop.Value;
            def.histop = v.x;
            def.histop2 = v.y;
            def.histop3 = v.z;
        }
        return def;
    }

    private OdeJointBallAndSocket CreateBallAndSocket(Transform tr, OdeBody body1, OdeBody body2)
    {
        if (tr == null)
            return null;

        var joint = tr.gameObject.AddComponent<OdeJointBallAndSocket>();
        joint.body = body1;
        joint.connectedBody = body2;
        joint.anchor = tr.position;
        //we need to remember the anchor position in parent (body1) local coordinates. The anchor lies at the pivot of rotation of child (body2)
        joint.localAnchor=Quaternion.Inverse(body1.rotation)*(joint.anchor-body1.position);
        return joint;
    }

    private OdeJointBallAndSocket CreateBallAndSocketWithAMotor(Transform tr, OdeBody body1, OdeBody body2)
    {
        if (tr == null)
            return null;

        OdeJointBallAndSocket joint = CreateBallAndSocket(tr, body1, body2);

        OdeJointAngularMotor motor = tr.gameObject.AddComponent<OdeJointAngularMotor>();
        motor.body = body1;
        motor.connectedBody = body2;
        motor.localAnchor = joint.localAnchor;
        motor.fudgeFactor = -1;
        motor.fudgeFactor2 = -1;
        motor.fudgeFactor3 = -1;

        motor.mode = OdeJointAngularMotor.AMotorMode.Euler;

        motor.nAxes = 3;
        //set some default axes. Note that Rig class overrides these in AddBone
        motor.SetAxis(0, 1, body2.transform.right);
        motor.SetAxis(2, 2, body2.transform.forward);

        motor.loStop = Mathf.Deg2Rad * this.lostop;
        motor.hiStop = Mathf.Deg2Rad * this.histop;

        motor.loStop2 = Mathf.Deg2Rad * this.lostop2;
        motor.hiStop2 = Mathf.Deg2Rad * this.histop2;

        motor.loStop3 = Mathf.Deg2Rad * this.lostop3;
        motor.hiStop3 = Mathf.Deg2Rad * this.histop3;

        float cfm, erp;
        CalculateDefaultMotorCfmErp(1 / 25.0f, out cfm, out erp);

        motor.cfm = motor.cfm2 = motor.cfm3 = cfm;

        motor.stopCfm = motor.stopCfm2 = motor.stopCfm3 = cfm;
        motor.stopErp = motor.stopErp2 = motor.stopErp3 = erp;


        return joint;
    }

    #endregion

    #region Hinge

    public static OdeJointDefinition CreateHinge(float loStop = float.NegativeInfinity,
        float hiStop = float.PositiveInfinity)
    {
        OdeJointDefinition def = new OdeJointDefinition();
        def.type = Type.Hinge;
        def.lostop = loStop;
        def.histop = hiStop;
        return def;
    }

    private OdeJointHinge CreateHinge(Transform tr, OdeBody body1, OdeBody body2)
    {
        if (tr == null)
            return null;

        var joint = tr.gameObject.AddComponent<OdeJointHinge>();
        joint.body = body1;
        joint.axesRel[0] = body1;
        joint.connectedBody = body2; 

        joint.anchor = tr.position;

        if (axis.HasValue)
            joint.axis = axis.Value;
        else
            joint.axis = joint.transform.forward;


        joint.loStop = Mathf.Deg2Rad * this.lostop;
        joint.hiStop = Mathf.Deg2Rad * this.histop;

        float cfm, erp;
        CalculateDefaultMotorCfmErp(1 / 25.0f, out cfm, out erp);

        joint.cfm = cfm;

        joint.stopCFM = cfm;
        joint.stopERP = erp;
        joint.nAxes = 1;
        joint.localAnchor=Quaternion.Inverse(body1.rotation)*(joint.anchor-body1.position);
        return joint;
    }

    public OdeJoint Create(Transform startTransform, OdeBody body, OdeBody connectedBody)
    {
        OdeJoint joint = null;
        switch (type)
        {
            case Type.Hinge:
                joint = CreateHinge(startTransform, body, connectedBody);
                break;
            case Type.BallAndSocket:
                joint = CreateBallAndSocketWithAMotor(startTransform, body, connectedBody);
                break;
            default:
                Debug.LogError("Could not create joint for definition of type: " + type.ToString());
                break;
        }

        if (joint != null)
            joint.def = this;

        return joint;
    }

    #endregion

    public static OdeJointDefinition CreateFixed()
    {
        OdeJointDefinition def = new OdeJointDefinition();
        def.type = Type.Fixed;
        return def;
    }

    

    public Type type;

    public Vector3? axis;
    public float angle;
    public float angleRate;
    public float position;
    public float positionRate;
    public Vector3 axis1;
    public Vector3 axis2;
    public float angle1;
    public float angle2;
    public float angle1Rate;
    public float angle2Rate;
    public Vector3 anchorDelta;
    public Vector3 axis3;
    public Vector3 axisDelta;
    public Vector3 force;
    public float lostop;
    public float histop;
    public float lostop2;
    public float histop2;
    public float lostop3;
    public float histop3;

    public float? erp;
    public float? cfm;
}

public class OdeJoint : MonoBehaviour
{
    public int debugBody1;
    public int debugBody2;
    public int nAxes = 0;
    //in the following two, "local" means in body 1 coordinates
    public Vector3[] localAxes = new Vector3[3]{Vector3.zero,Vector3.zero,Vector3.zero};
    public Vector3 localAnchor;
    public OdeBody[] axesRel = new OdeBody[3] { null, null, null };
    public bool disableUpdate { get; set; }
    public Quaternion initialConnectedBodyLocalRotation = Quaternion.identity;
    public Vector3 initialConnectedBodyLocalPosition = Vector3.zero;
    private OdeBody m_body = null;

    [HideInInspector]
    public OdeJointDefinition def;

    public OdeBody body
    {
        get
        {
            return m_body;
        }
        set
        {
            m_body = value;
            debugBody1 = m_body.BodyId;
        }
    }

    private OdeBody m_connectedBody = null;
    public OdeBody connectedBody
    {
        get { return m_connectedBody; }
        set
        {
            m_connectedBody = value;
            if (m_connectedBody != null)
            {
                //Debug.Log("Connecting bodies " + body.BodyId + " and " + connectedBody.BodyId + " with joint " + jointId);
                UnityOde.odeJointAttach(jointId, body.BodyId, connectedBody.BodyId);
                debugBody2 = m_connectedBody.BodyId;
                initialConnectedBodyLocalRotation = Quaternion.Inverse(body.rotation) * connectedBody.rotation;
                initialConnectedBodyLocalPosition = Quaternion.Inverse(body.rotation) * (connectedBody.position - body.position);
            }
        }
    }

    private int jointId = -1;
    public int JointId
    {
        get { return jointId; }
        protected set { jointId = value; }
    }
    protected virtual void FixedUpdate()
    {
    }
    protected int myContext = 0;
    protected virtual void Awake()
    {
        myContext = UnityOde.getCurrentOdeContext();
    }

    protected virtual void Start()
    {
        if (body != null)
        {
            
            body.LocalOffset = body.transform.InverseTransformPoint(transform.position); // joint position in body space
        }
    }


    public static bool drawGizmos = true;

    protected void OnDrawGizmos()
    {
        if (!drawGizmos)
        {
            return;
        }

        if (JointId < 0)
            return;
        UnityOde.setCurrentOdeContext(myContext);
        int body1 = UnityOde.odeJointGetBody(JointId, 0);
        int body2 = UnityOde.odeJointGetBody(JointId, 1);

        if (body1 > 0 && body2 > 0)
        {
			Vector3 v1 = UnityOde.odeBodyGetPosition(body1);
			Vector3 v2 = UnityOde.odeBodyGetPosition(body2);

            Gizmos.color = Color.red;
            Gizmos.DrawLine(v1, v2);
        }
    }

    protected void OnDrawGizmosSelected()
    {
        if (JointId < 0)
            return;
        UnityOde.setCurrentOdeContext(myContext);

        int body1 = UnityOde.odeJointGetBody(JointId, 0);
        int body2 = UnityOde.odeJointGetBody(JointId, 1);

        if (body1 > 0 && body2 > 0)
        {
			Vector3 loc1 = UnityOde.odeBodyGetPosition(body1);
			Vector3 loc2 = UnityOde.odeBodyGetPosition(body2);

            Gizmos.color = Color.red;
            Gizmos.DrawCube(loc1, new Vector3(0.05f, 0.05f, 0.05f));
            Gizmos.DrawCube(loc2, new Vector3(0.05f, 0.05f, 0.05f));
        }
    }

	public override bool Equals(System.Object obj)
	{
		OdeJoint p = obj as OdeJoint;
		if ((object)p == null)
			return false;
		return base.Equals(obj);
	}

	public override int GetHashCode()
	{
		return base.GetHashCode();
	}

	public static bool operator ==(OdeJoint a, OdeJoint b)
	{
		if (System.Object.ReferenceEquals(a, b))
			return true;
		if (((object)a == null) || ((object)b == null))
			return false;
		return a.Equals(b);
	}

	public static bool operator !=(OdeJoint a, OdeJoint b)
	{
		return !(a == b);
	}

    // TODO: these should be removed
    public virtual Vector3? getAnchor()
    {
        return null;
    }

    public virtual void setAnchor(Vector3 anchor) { }
}
