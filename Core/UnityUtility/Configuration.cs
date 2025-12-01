using System;
using UnityEngine;
using System.Runtime.InteropServices;
using System.Text;

namespace CsCore
{
    public class Configuration
    {
        [DllImport("kernel32", CharSet = CharSet.Unicode)]
        static extern long WritePrivateProfileString(
            string Section, string Key, string Value, string FilePath);
        [DllImport("kernel32", CharSet = CharSet.Unicode)]
        static extern int GetPrivateProfileString(
            string Section, string Key, string Default, StringBuilder RetVal, int Size, string FilePath);
        //public static string ReadConfigIni(string section, string key)pjh
        public static string ReadConfigIni(string section, string key, string defaultValue = "")
        {
            var value = new StringBuilder(255);
            
            //pjh
            //GetPrivateProfileString(section, key, "", value, 255, UnityEngine.Application.dataPath + "/CsConfig.ini");
            GetPrivateProfileString(section, key, defaultValue, value, 255, UnityEngine.Application.dataPath + "/CsConfig.ini");
            if (value.ToString() == defaultValue || string.IsNullOrEmpty(value.ToString())) WriteConfigIni(section, key, defaultValue);
            //

            return value.ToString();
        }
        public static void WriteConfigIni(string section, string key, string value)
        {
            WritePrivateProfileString(section, key, value, UnityEngine.Application.dataPath + "/CsConfig.ini");
        }
        //pjh
        public static Vector3 StringToVector3(string str)
        {
            string[] parts = str.Split(',');
            if (parts.Length != 3)
            {
                throw new FormatException("Invalid Vector3 format");
            }

            float x = float.Parse(parts[0]);
            float y = float.Parse(parts[1]);
            float z = float.Parse(parts[2]);

            return new Vector3(x, y, z);
        }
        //
    }
}
