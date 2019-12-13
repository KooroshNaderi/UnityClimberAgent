using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Runtime.InteropServices;
using AaltoGames;
using System.IO;
using System.Reflection;
using System.Xml.Serialization;

 
/// <summary>
/// Static utility class to work around lack of support for Keyframe.tangentMode
/// This utility class mimics the functionality that happens behind the scenes in UnityEditor when you manipulate an AnimationCurve. All of this information
/// was discovered via .net reflection, and thus relies on reflection to work
/// --testure 09/05/2012
/// </summary>
using System.Text;


public static class AnimationCurveUtility : System.Object
{
    public enum TangentMode
    {
        Editable,
        Smooth,
        Linear,
        Stepped
    }
 
    public enum TangentDirection 
    {
        Left,
        Right
    }
 
    public static void SetTangentMode( AnimationCurve curve , TangentMode mode)  //was using ref...
    {
        Type t = typeof( UnityEngine.Keyframe );
        FieldInfo field = t.GetField( "m_TangentMode", BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.Instance );
 
        for ( int i = 0; i < curve.length; i++ )
        {
            object boxed = curve.keys[ i ]; // getting around the fact that Keyframe is a struct by pre-boxing
            field.SetValue( boxed, GetNewTangentKeyMode( ( int ) field.GetValue( boxed ), TangentDirection.Left, mode ) );
            field.SetValue( boxed, GetNewTangentKeyMode( ( int ) field.GetValue( boxed ), TangentDirection.Right, mode ) );
            curve.MoveKey( i, ( Keyframe ) boxed );
            curve.SmoothTangents( i, 0f );
        }
    }
 
    public static int GetNewTangentKeyMode( int currentTangentMode, TangentDirection leftRight, TangentMode mode )
    {
        int output = currentTangentMode;
 
        if ( leftRight == TangentDirection.Left )
        {
            output &= -7;
            output |= ( ( int ) mode ) << 1;
        }
        else
        {
            output &= -25;
            output |= ( ( int ) mode ) << 3;
        }
        return output;
    }
}

public static class ODEUtils
{
    public static void restoreOdeStateCheckValue()
    {
        if (!UnityOde.restoreOdeState())
        {
            Debug.LogException(new Exception("Cannot restore ODE state"));
        }
    }
}

public static class Utils
{
    public static float sqrVectorMagnitude(this Quaternion quat)
    {
        return quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w;
    }

    public static Quaternion getPoseMirrored(this Quaternion quat)
    {
        // Inverse and change z direction
        return new Quaternion(quat.x, -quat.y, quat.z, -quat.w);
    }
    public static Transform findRecursive(Transform parent, string name)
    {
        Transform result = parent.Find(name);
        if (result != null)
            return result;
        foreach (Transform t in parent)
        {
            Transform child = findRecursive(t, name);
            if (child != null)
                return child;
        }
        return null;
    }
    public static void disableRenderersRecursive(Transform t)
    {
        foreach (Transform child in t)
        {
            disableRenderersRecursive(child);
        }
        SkinnedMeshRenderer r = t.GetComponent<SkinnedMeshRenderer>();
        if (r != null)
            r.enabled = false;
    }
    public static void enableRenderersRecursive(Transform t)
    {
        foreach (Transform child in t)
        {
            enableRenderersRecursive(child);
        }
        SkinnedMeshRenderer r = t.GetComponent<SkinnedMeshRenderer>();
        if (r != null)
            r.enabled = true;
    }
    public static void setLayerRecursive(Transform transform, int layer)
    {
        foreach (Transform child in transform)
        {
            setLayerRecursive(child, layer);
        }

        if (transform.gameObject != null)
            transform.gameObject.layer = layer;
    }
    public static void setAlphaRecursive(Transform transform, float alpha)
    {
        foreach (Transform child in transform)
        {
            setAlphaRecursive(child, alpha);
        }
        SkinnedMeshRenderer r=transform.GetComponent<SkinnedMeshRenderer>();
        if (r != null)
        {
            Color color=r.material.color;
            color.a=alpha;
            r.material.color=color;
        }
    }
    public static void copyTransformsRecursive(Transform src, Transform dst)
    {
        for (int i = 0; i < src.childCount; i++)
        {
            Transform srcChild = src.GetChild(i);
            Transform dstChild = dst.Find(src.GetChild(i).name);
            if (dstChild!=null)
                copyTransformsRecursive(srcChild,dstChild);
        }
        dst.localPosition = src.localPosition;
        dst.localRotation = src.localRotation;
    }
    //soft threshold so that minThreshold gives 0.01f and maxThreshold gives 0.99f
    public static double dualSoftThresholdGt(double value, double minThreshold, double maxThreshold)
    {
        const double atan98 = 2.29756;
        double threshold = 0.5 * (minThreshold + maxThreshold);
        double halfRange = 0.5 * (maxThreshold - minThreshold);
        return 0.5 + 0.5 * Math.Tanh((value - threshold) / halfRange * atan98);

    }
    //soft threshold so that minThreshold gives 0.99f and maxThreshold gives 0.01f
    public static double dualSoftThresholdLt(double value, double minThreshold, double maxThreshold)
    {
        const double atan98 = 2.29756;
        double threshold = 0.5 * (minThreshold + maxThreshold);
        double halfRange = 0.5 * (maxThreshold - minThreshold);
        return 0.5 + 0.5 * Math.Tanh((threshold - value) / halfRange * atan98);
    }

}


[Serializable]
public class SerializableVector3
{
    public float X;
    public float Y;
    public float Z;
    public SerializableVector3()
    {
        X = 0;
        Y = 0;
        Z = 0;
    }
    public SerializableVector3(float x=0, float y=0, float z=0)
    {
        X = x;
        Y = y; 
        Z = z;
    }
    public static implicit operator Vector3(SerializableVector3 sv)
    {
        return new Vector3(sv.X, sv.Y, sv.Z);
    }
    public static implicit operator SerializableVector3(Vector3 v)
    {
        return new SerializableVector3(v.x, v.y, v.z);
    }
}

[Serializable]
public class SerializableQuaternion
{
    public float X;
    public float Y;
    public float Z;
    public float W;

    public SerializableQuaternion()
    {
        X = 0;
        Y = 0;
        Z = 0;
        W = 1;
    }
    public SerializableQuaternion(float x=0, float y=0, float z=0, float w=1)
    {
        X = x;
        Y = y; 
        Z = z;
        W = w;
    }
    public static implicit operator Quaternion(SerializableQuaternion sv)
    {
        return new Quaternion(sv.X, sv.Y, sv.Z,sv.W);
    }
    public static implicit operator SerializableQuaternion(Quaternion v)
    {
        return new SerializableQuaternion(v.x, v.y, v.z,v.w);
    }
}
