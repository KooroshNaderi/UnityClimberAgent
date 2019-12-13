using System;
using UnityEngine;

public class OdeJointHinge : OdeJoint
{
    public Vector3 anchor
    {
        get
        {
            Vector3 v;
            UnityOde.odeJointGetHingeAnchor(JointId, out v.x, out v.y, out v.z);
            return v;
        }
        set
        {
            UnityOde.odeJointSetHingeAnchor(JointId, value.x, value.y, value.z);
        }
    }

    public float loStop
    {
        get
        {
            return UnityOde.odeJointGetHingeParam(JointId, (int)OdeWrapper.OdeParam.LoStop);
        }
        set
        {
            UnityOde.odeJointSetHingeParam(JointId, (int)OdeWrapper.OdeParam.LoStop, value);
        }
    }

    public float hiStop
    {
        get
        {
            return UnityOde.odeJointGetHingeParam(JointId, (int)OdeWrapper.OdeParam.HiStop);
        }
        set
        {
            UnityOde.odeJointSetHingeParam(JointId, (int)OdeWrapper.OdeParam.HiStop, value);
        }
    }

    public float fmax
    {
        get
        {
            return UnityOde.odeJointGetHingeParam(JointId, (int)OdeWrapper.OdeParam.FMax);
        }
        set
        {
            UnityOde.odeJointSetHingeParam(JointId, (int)OdeWrapper.OdeParam.FMax, value);
        }
    }

    public float fudgeFactor
    {
        get
        {
            return UnityOde.odeJointGetHingeParam(JointId, (int)OdeWrapper.OdeParam.FudgeFactor);
        }
        set
        {
            UnityOde.odeJointSetHingeParam(JointId, (int)OdeWrapper.OdeParam.FudgeFactor, value);
        }
    }

    public float stopERP
    {
        get
        {
            return UnityOde.odeJointGetHingeParam(JointId, (int)OdeWrapper.OdeParam.StopERP);
        }
        set
        {
            UnityOde.odeJointSetHingeParam(JointId, (int)OdeWrapper.OdeParam.StopERP, value);
        }
    }

    public float stopCFM
    {
        get
        {
            return UnityOde.odeJointGetHingeParam(JointId, (int)OdeWrapper.OdeParam.StopCFM);
        }
        set
        {
            UnityOde.odeJointSetHingeParam(JointId, (int)OdeWrapper.OdeParam.StopCFM, value);
        }
    }

    public float cfm
    {
        get
        {
            return UnityOde.odeJointGetHingeParam(JointId, (int)OdeWrapper.OdeParam.CFM);
        }
        set
        {
            UnityOde.odeJointSetHingeParam(JointId, (int)OdeWrapper.OdeParam.CFM, value);
        }
    }
    public float erp
    {
        get
        {
            return UnityOde.odeJointGetHingeParam(JointId, (int)OdeWrapper.OdeParam.ERP);
        }
        set
        {
            UnityOde.odeJointSetHingeParam(JointId, (int)OdeWrapper.OdeParam.ERP, value);
        }
    }
    
    
    public override Vector3? getAnchor()
    {
        Vector3 v = new Vector3();
        UnityOde.odeJointGetHingeAnchor(JointId, out v.x, out v.y, out v.z);
        return v;
    }

    public override void setAnchor(Vector3 anchor)
    {
        this.anchor = anchor;
    }

    public Vector3 axis
    {
        get
        {
            Vector3 v = new Vector3();
            UnityOde.odeJointGetHingeAxis(JointId, out v.x, out v.y, out v.z);
            return v;
        }
        set
        {
            UnityOde.odeJointSetHingeAxis(JointId, value.x, value.y, value.z);
            localAxes[0] = Quaternion.Inverse(axesRel[0].rotation) * value;
        }
    }

    protected override void Awake() 
    {
        JointId = UnityOde.odeJointCreateHinge();
        //Debug.Log("Created hinge joint: " + name);

        base.Awake();
    }

    protected override void Start()
    {
//        transform.position = anchor;

        base.Start();
    }

    protected override void FixedUpdate()
    {
        if (!disableUpdate)
        {
            transform.position = anchor;
        }

        base.FixedUpdate();
    }

    new void OnDrawGizmosSelected()
    {
        if (!drawGizmos)
        {
            return;
        }
        UnityOde.setCurrentOdeContext(myContext);

        Gizmos.color = Color.white;
        Gizmos.DrawWireSphere(anchor, 0.05f);
        Gizmos.DrawLine(anchor, anchor + axis);

        base.OnDrawGizmosSelected();
    }

}
