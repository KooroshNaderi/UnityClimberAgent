using System;
using UnityEngine;

public abstract class OdeGeom : MonoBehaviour
{
    public int debugId;

    private int geomId = -1;
    public int GeomId
    {
        get { return geomId; }
        protected set { geomId = debugId = value; }
    } 

    /*public Vector3 center
    {
        set
        {
            UnityOde.odeGeomSetOffsetWorldPosition(GeomId, value.x, value.y, value.z);
        }
    }*/

    private OdeBody body = null;
    public OdeBody Body
    {
        get { return body; }
        set
        {
            // Check for changes
            if (body != value)
            {
                // Detach old
                if (body != null)
                {
                    UnityOde.odeGeomSetBody(GeomId, 0);
                }
                body = value;
                // Attach new
                if (body != null)
                {
                    //Debug.Log("Combining geom " + geomId + " and body " + body.BodyId);
                    UnityOde.odeGeomSetBody(GeomId, body.BodyId);
                }
            }
        }
    }

    public Vector3 position
    {
        get
        {
            return UnityOde.odeGeomGetPosition(GeomId);
        }
        set
        {
            UnityOde.odeGeomSetPosition(GeomId, value.x, value.y, value.z);
        }
    }

    public Quaternion rotation
    {
        get
        {
            Quaternion q;
            UnityOde.odeGeomGetQuaternion(GeomId, out q);
            return q;
        }
        set
        {
            UnityOde.odeGeomSetQuaternion(GeomId, value);
        }
    }
    protected int myContext = 0;
    protected virtual void Awake()
    {
        myContext = UnityOde.getCurrentOdeContext();
        // Disable all Unity colliders
        if (GetComponent<Collider>() != null)
            GetComponent<Collider>().enabled = false;

        OdeBody body = null;
        if ((body = GetComponent<OdeBody>()) != null)
        {
            if (body.BodyId != -1)
                Body = body;
        }
    }

    protected virtual void Start()
    {
        //position = transform.position;
        //rotation = transform.rotation;
    }

    void OnDestroy()
    {
        UnityOde.setCurrentOdeContext(myContext);
        if (UnityOde.initialized())
            UnityOde.odeGeomDestroy(GeomId);
    }

    protected virtual void FixedUpdate()
    {
        //transform.position = position;
        //transform.rotation = rotation;
    }

    public abstract void SetMass(float mass);
    public abstract void SetDensity(float density);
}
