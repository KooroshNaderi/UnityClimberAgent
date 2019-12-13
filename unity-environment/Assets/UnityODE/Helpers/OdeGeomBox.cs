using System;
using UnityEngine;

class OdeGeomBox : OdeGeom
{
    public Vector3 lengths
    {
        get
        {
            float x, y, z;
            UnityOde.odeGeomBoxGetLengths(GeomId, out x, out y, out z);
            return new Vector3(x, y, z);
        }
        set
        {
            UnityOde.odeGeomBoxSetLengths(GeomId, value.x, value.y, value.z);
        }
    }

    public static bool debug = false;
    private GameObject debugBox;

    protected override void Awake()
    {
        base.GeomId = UnityOde.odeCreateBox(1, 1, 1);

        if (debug)
        {
            debugBox = GameObject.CreatePrimitive(PrimitiveType.Cube);
            debugBox.name = "dbgBox_" + gameObject.name;
            debugBox.transform.parent = transform;
            debugBox.transform.localPosition = Vector3.zero;
        }

        base.Awake();
    }

    protected override void FixedUpdate()
    {
        if (debugBox != null)
        {
            if (debug)
            {
                UnityOde.setCurrentOdeContext(myContext);
                if (Body != null)
                {
                    debugBox.transform.rotation = Body.rotation;
                    debugBox.transform.position = Body.position;
                }
                else
                {
                    debugBox.transform.rotation = rotation;
                    debugBox.transform.position = position;
                }
               
                debugBox.transform.localScale = lengths;
                debugBox.SetActive(true);
            }
            else
            {
                debugBox.SetActive(false);
            }
        }
    }

    public override void SetMass(float mass)
    {
        if (Body != null)
        {
            Vector3 lengths = this.lengths;
            UnityOde.odeMassSetBoxTotal(Body.BodyId, mass, lengths.x, lengths.y, lengths.z);
        }
        else
        {
            Debug.LogError("Could not set mass to " + gameObject.name + " because there is no body attached");
        }
    }

    public override void SetDensity(float density)
    {
        if (Body != null)
        {
            Vector3 lengths = this.lengths;
            UnityOde.odeMassSetBox(Body.BodyId, density, lengths.x, lengths.y, lengths.z);
        }
        else
            Debug.LogError("Could not set mass to " + gameObject.name + " because there is no body attached");
    }
}
