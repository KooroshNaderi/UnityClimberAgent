using System;
using System.Runtime.InteropServices;
using UnityEngine;

//some extra wrapping stuff
public static class OdeWrapper
{
	//type conversions between Ode and Unity quaternions and vectors
	/*public static Vector3 oconv(this OdeVec3 src)
	{
		return new Vector3(src.x,src.y,src.z);
	}
	public static Vector4 oconv(this OdeVec4 src)
	{
		return new Vector4(src.x,src.y,src.z,src.w);
	}
	public static Quaternion oconv(this OdeQuat src)
	{ 
		return new Quaternion(src.x,src.y,src.z,src.w);
	}
	public static OdeVec3 oconv(this Vector3 src)
	{
		return new OdeVec3(src.x,src.y,src.z);
	}
	public static OdeVec4 oconv(this Vector4 src)
	{
		return new OdeVec4(src.x,src.y,src.z,src.w);
	}
	public static OdeQuat oconv(this Quaternion src)
	{
		return new OdeQuat(src.x,src.y,src.z,src.w);
	}*/
	
	//type conversions between SWIG pointers and IntPtrs
	public static IntPtr ToIntPtr(this SWIGTYPE_p_float src)
	{
		return SWIGTYPE_p_float.getCPtr(src).Handle;
	}
	
    // TODO: these should actually use values from ODE API
    internal enum OdeParam
    {
        LoStop = 0, HiStop, Vel, LoVel, HiVel, FMax, FudgeFactor, Bounce, CFM, StopERP, StopCFM, SuspensionERP, SuspensionCFM, ERP,
        LoStop1 = 0x000, HiStop1, Vel1, LoVel1, HiVel1, FMax1, FudgeFactor1, Bounce1, CFM1, StopERP1, StopCFM1, SuspensionERP1, SuspensionCFM1, ERP1,
        LoStop2 = 0x100, HiStop2, Vel2, LoVel2, HiVel2, FMax2, FudgeFactor2, Bounce2, CFM2, StopERP2, StopCFM2, SuspensionERP2, SuspensionCFM2, ERP2,
        LoStop3 = 0x200, HiStop3, Vel3, LoVel3, HiVel3, FMax3, FudgeFactor3, Bounce3, CFM3, StopERP3, StopCFM3, SuspensionERP3, SuspensionCFM3, ERP3
    }
    internal enum AMotorMode
    {
        User = 0,
        Euler
    }
    internal enum dJointType
    {
        dJointTypeAny = -1, //for queries filtered by joint type
        dJointTypeNone = 0,		/* or "unknown" */
        dJointTypeBall,
        dJointTypeHinge,
        dJointTypeSlider,
        dJointTypeContact,
        dJointTypeUniversal,
        dJointTypeHinge2,
        dJointTypeFixed,
        dJointTypeNull,
        dJointTypeAMotor,
        dJointTypeLMotor,
        dJointTypePlane2D,
        dJointTypePR,
        dJointTypePU,
        dJointTypePiston
    }


}

//helper class for dealig with pinned arrays
public class PinnedArray<T>
{
	public T [] arr;
	GCHandle pinnedHandle;
	public PinnedArray(int size)
	{
		arr=new T[size];
		pinnedHandle=GCHandle.Alloc(arr, GCHandleType.Pinned);
	}
	public PinnedArray(T [] srcArr)
	{
		arr=srcArr;
		pinnedHandle=GCHandle.Alloc(arr, GCHandleType.Pinned);		
	}
	public T this[int idx]
	{
		get{
			return arr[idx];
			
		}
		set{
			arr[idx]=value;
		}
	}
    public static implicit operator SWIGTYPE_p_int(PinnedArray<T> src)
    {
        if (src == null)
            return null;
        return new SWIGTYPE_p_int(src.pinnedHandle.AddrOfPinnedObject(), false);
    }
    public static implicit operator SWIGTYPE_p_float(PinnedArray<T> src)
    {
        if (src == null)
            return null;
        return new SWIGTYPE_p_float(src.pinnedHandle.AddrOfPinnedObject(),false);
    }
    //public static implicit operator SWIGTYPE_p_double(PinnedArray<T> src)
    //{
    //    return new SWIGTYPE_p_double(src.pinnedHandle.AddrOfPinnedObject(),false);
	//}
}
