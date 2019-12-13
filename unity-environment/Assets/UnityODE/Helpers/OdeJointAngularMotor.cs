using System;
using UnityEngine;

public class OdeJointAngularMotor :  OdeJoint
{
    public enum AMotorMode
    {
        User = OdeWrapper.AMotorMode.User,
        Euler = OdeWrapper.AMotorMode.Euler
    }

    public AMotorMode mode
    {
        get
        {
            return (AMotorMode)UnityOde.odeJointGetAMotorMode(JointId);
        }
        set 
        {
            UnityOde.odeJointSetAMotorMode(JointId, (int)value);
        }
    }

    public int numAxes
    {
        get { return UnityOde.odeJointGetAMotorNumAxes(JointId); }
        set { UnityOde.odeJointSetAMotorNumAxes(JointId, value); }
    }

    public Vector3 angle
    {
        get
        {
            Vector3 v;
            v.x = UnityOde.odeJointGetAMotorAngle(JointId, 0);
            v.y = UnityOde.odeJointGetAMotorAngle(JointId, 1);
            v.z = UnityOde.odeJointGetAMotorAngle(JointId, 2);
            return v;
        }
        set
        {
            UnityOde.odeJointSetAMotorAngle(JointId, 0, value.x);
            UnityOde.odeJointSetAMotorAngle(JointId, 1, value.y);
            UnityOde.odeJointSetAMotorAngle(JointId, 2, value.z);
        }
    }

    public float loStop
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.LoStop); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.LoStop, value); }
    }

    public float hiStop
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.HiStop); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.HiStop, value); }
    }

    public float vel
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.Vel); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.Vel, value); }
    }

    public float fmax
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.FMax); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.FMax, value); }
    }

    public float fudgeFactor
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.FudgeFactor); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.FudgeFactor, value); }
    }

    public float bounce
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.Bounce); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.Bounce, value); }
    }

    public float cfm
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.CFM); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.CFM, value); }
    }
    public float erp
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.ERP); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.ERP, value); }
    }

    public float stopErp
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.StopERP); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.StopERP, value); }
    }

    public float stopCfm
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.StopCFM); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.StopCFM, value); }
    }

    public float loStop2
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.LoStop2); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.LoStop2, value); }
    }

    public float hiStop2
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.HiStop2); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.HiStop2, value); }
    }

    public float vel2
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.Vel2); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.Vel2, value); }
    }

    public float fmax2
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.FMax2); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.FMax2, value); }
    }

    public float fudgeFactor2
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.FudgeFactor2); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.FudgeFactor2, value); }
    }

    public float bounce2
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.Bounce2); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.Bounce2, value); }
    }

    public float cfm2
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.CFM2); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.CFM2, value); }
    }
    public float erp2
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.ERP2); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.ERP2, value); }
    }

    public float stopErp2
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.StopERP2); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.StopERP2, value); }
    }

    public float stopCfm2
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.StopCFM2); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.StopCFM2, value); }
    }

    public float loStop3
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.LoStop3); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.LoStop3, value); }
    }

    public float hiStop3
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.HiStop3); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.HiStop3, value); }
    }

    public float vel3
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.Vel3); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.Vel3, value); }
    }

    public float fmax3
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.FMax3); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.FMax3, value); }
    }

    public float fudgeFactor3
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.FudgeFactor3); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.FudgeFactor3, value); }
    }

    public float bounce3
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.Bounce3); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.Bounce3, value); }
    }

    public float cfm3
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.CFM3); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.CFM3, value); }
    }

    public float erp3
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.ERP3); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.ERP3, value); }
    }
    public float stopErp3
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.StopERP3); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.StopERP3, value); }
    }

    public float stopCfm3
    {
        get { return UnityOde.odeJointGetAMotorParam(JointId, (int)OdeWrapper.OdeParam.StopCFM3); }
        set { UnityOde.odeJointSetAMotorParam(JointId, (int)OdeWrapper.OdeParam.StopCFM3, value); }
    }

    // TODO: Change anum and rel to enums
    public void SetAxis(int anum, int rel, Vector3 axis)
    {
        UnityOde.odeJointSetAMotorAxis(JointId, anum, rel, axis.x, axis.y, axis.z);

        if (rel == 1)
        {
            axesRel[anum] = body;
        }
        else if (rel == 2)
        {
            axesRel[anum] = connectedBody;
        }
        else
        {
            Debug.LogException(new Exception("Not supported!"));
        }
        localAxes[anum] = Quaternion.Inverse(body.rotation) * axis; //we always use body here, as motorAnglesToPose() needs to be able to figure out the skeleton hierarchically -> global axis definition must not depend on child (connectedBody)
    }

    Vector3 GetAxis(int anum)
    {
        Vector3 v = new Vector3();
        UnityOde.odeJointGetAMotorAxis(JointId, anum, out v.x, out v.y, out v.z);
        return v;
    }

    int GetAxisRel(int anum)
    {
        return UnityOde.odeJointGetAMotorAxisRel(JointId, anum);
    }

    protected override void Awake()
    {

        JointId = UnityOde.odeJointCreateAMotor();

        base.Awake();
    }

    new void OnDrawGizmosSelected()
    {
        UnityOde.setCurrentOdeContext(myContext);

        Vector3 axis0 = GetAxis(0);
        Vector3 axis1 = GetAxis(1);
        Vector3 axis2 = GetAxis(2);

        axis1 = Vector3.Cross(axis0, axis2);

        Vector3 p = transform.position;

        Gizmos.color = Color.red;
        Gizmos.DrawLine(p, p + axis0);
        Gizmos.color = Color.green;
        Gizmos.DrawLine(p, p + axis1);
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(p, p + axis2);

        base.OnDrawGizmosSelected();
    }
}
