using UnityEngine;
using System.Collections;
using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using UnityEditor;

public class ImportUnityRRT : MonoBehaviour {
    //Editor stuff
	// Helper for copying the built tracker dll to the plugins folder
	[MenuItem("UnityODE/Reimport DLL and wrappers")]
	static void importDll()
	{
        string path=EditorUtility.OpenFolderPanel("Select UnityODE Visual Studio dll folder",EditorPrefs.GetString("UnityODEDllFolder"),"");
        EditorPrefs.SetString("UnityODEDllFolder", path);
        //copy the ODE dll if it has been built
        if (!tryCopy(path+"\\dll\\UnityOde.dll", "Assets/UnityODE/Plugins/UnityOde.dll"))
        {
            Debug.LogError("Cannot copy the ODE dll, check that you've built it!");
            return;
        }
        //copy debug info if it exists
        tryCopy(path + "UnityOde.pdb", "Assets/UnityODE/Plugins/UnityOde.pdb");

        //copy all .cs files

        DirectoryInfo src = new DirectoryInfo(path);
        DirectoryInfo dst = new DirectoryInfo("Assets/UnityODE/OdeWrapper");
        CopyFiles(src, dst, true, "*.cs");

        Debug.Log("Copied the DLL and .cs files");
    }

    static bool tryCopy(string dllName, string dstName)
	{
		if (File.Exists(dllName))
		{
			File.Copy(dllName,dstName,true);
			return true;
		}
		return false;
	}
    static void CopyFiles(DirectoryInfo source,
                          DirectoryInfo destination,
                          bool overwrite,
                          string searchPattern)
    {
        FileInfo[] files = source.GetFiles(searchPattern);

        //this section is what's really important for your application.
        foreach (FileInfo file in files)
        {
            file.CopyTo(destination.FullName + "\\" + file.Name, overwrite);
        }
    }
    // Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
	
	}
}
