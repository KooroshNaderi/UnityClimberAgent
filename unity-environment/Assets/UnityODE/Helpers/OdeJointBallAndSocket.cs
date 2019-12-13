using System;
using UnityEngine;

public class OdeJointBallAndSocket : OdeJoint
{
    public Vector3 anchor
    {
        get
        {
            Vector3 v = new Vector3();
            UnityOde.odeJointGetBallAnchor(JointId, out v.x, out v.y, out v.z);
            return v;
        }
        set
        {
            UnityOde.odeJointSetBallAnchor(JointId, value.x, value.y, value.z);
        }
    }

    public float cfm
    {
        get { return UnityOde.odeJointGetBallParam(JointId, (int)OdeWrapper.OdeParam.CFM); }
        set { UnityOde.odeJointSetBallParam(JointId, (int)OdeWrapper.OdeParam.CFM, value); }
    }
    public float cfm2
    {
        get { return UnityOde.odeJointGetBallParam(JointId, (int)OdeWrapper.OdeParam.CFM2); }
        set { UnityOde.odeJointSetBallParam(JointId, (int)OdeWrapper.OdeParam.CFM2, value); }
    }
    public float cfm3
    {
        get { return UnityOde.odeJointGetBallParam(JointId, (int)OdeWrapper.OdeParam.CFM3); }
        set { UnityOde.odeJointSetBallParam(JointId, (int)OdeWrapper.OdeParam.CFM3, value); }
    }

    public float erp
    {
        get { return UnityOde.odeJointGetBallParam(JointId, (int)OdeWrapper.OdeParam.ERP); }
        set { UnityOde.odeJointSetBallParam(JointId, (int)OdeWrapper.OdeParam.ERP, value); }
    }
    public float erp2
    {
        get { return UnityOde.odeJointGetBallParam(JointId, (int)OdeWrapper.OdeParam.ERP2); }
        set { UnityOde.odeJointSetBallParam(JointId, (int)OdeWrapper.OdeParam.ERP2, value); }
    }
    public float erp3
    {
        get { return UnityOde.odeJointGetBallParam(JointId, (int)OdeWrapper.OdeParam.ERP3); }
        set { UnityOde.odeJointSetBallParam(JointId, (int)OdeWrapper.OdeParam.ERP3, value); }
    }

    protected override void Awake()
    {

        JointId = UnityOde.odeJointCreateBall();

        base.Awake();
    }


    protected override void FixedUpdate()
    {
        //transform.position = anchor;
    }
     
    new void OnDrawGizmos()
    {
        if (!drawGizmos)
        {
            return;
        }
        UnityOde.setCurrentOdeContext(myContext);

        Gizmos.color = Color.grey;
        Gizmos.DrawWireSphere(anchor, 0.05f);

        base.OnDrawGizmos();
    }

    new void OnDrawGizmosSelected()
    {
        UnityOde.setCurrentOdeContext(myContext);

        Gizmos.color = Color.white;
        Gizmos.DrawWireSphere(anchor, 0.05f);

        base.OnDrawGizmosSelected();
    }
}
