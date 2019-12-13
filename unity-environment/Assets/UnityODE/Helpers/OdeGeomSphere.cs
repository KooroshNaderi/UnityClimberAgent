using System;
using UnityEngine;

class OdeGeomSphere : OdeGeom
{
    public float radius
    {
        get
        {
            return UnityOde.odeGeomSphereGetRadius(GeomId);
        }
        set
        {
            UnityOde.odeGeomSphereSetRadius(GeomId, value);
            if (debug)
            {
                float radiusF = radius / 0.5f;
                Vector3 localScale = new Vector3(radiusF, radiusF, radiusF);
                debugSphere.transform.localScale = localScale;
            }
        }
    }

    public bool debug = true;
    public GameObject debugSphere;

    protected override void Awake()
    {
        base.GeomId = UnityOde.odeCreateSphere(0.5f);

        if (debug)
        {
            debugSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            debugSphere.transform.parent = transform;
            debugSphere.transform.localPosition = Vector3.zero;
            debugSphere.transform.localRotation = Quaternion.Euler(90.0f, 0.0f, 0.0f);
        }

        base.Awake();
    }
    protected override void FixedUpdate()
    {
        if (debug && debugSphere != null)
        {
            UnityOde.setCurrentOdeContext(myContext);
            // default size of Unity sphere primitive is r=0.5
            float radiusF = radius / 0.5f;
            Vector3 localScale = new Vector3(radiusF, radiusF, radiusF);

            debugSphere.transform.localScale = localScale;
        }
    }

    public override void SetMass(float mass)
    {
        if (Body != null)
            UnityOde.odeMassSetSphereTotal(Body.BodyId, mass, radius);
        else
            Debug.LogError("Could not set mass to " + gameObject.name + " because there is no body attached");
    }

    public override void SetDensity(float density)
    {
        if (Body != null)
            UnityOde.odeMassSetSphere(Body.BodyId, density, radius);
        else
            Debug.LogError("Could not set mass to " + gameObject.name + " because there is no body attached");
    }
}
