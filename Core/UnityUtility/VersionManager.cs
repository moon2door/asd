//using UnityEditor;
//using UnityEngine;

//[InitializeOnLoad]
//public class VersionManager
//{
//    private static bool AutoIncrease = true;
//    private const string AutoIncreaseMenuName = "Build/Auto Increase Build Version";

//    static VersionManager()
//    {
//        AutoIncrease = EditorPrefs.GetBool(AutoIncreaseMenuName, true);
//    }

//    [InitializeOnLoadMethod]
//    static void CheckVersionLength()
//    {
//        string[] lines = PlayerSettings.bundleVersion.Split('.');
//        if (lines.Length < 3)
//        {
//            int MajorVersion = 0;
//            int MinorVersion = 0;
//            int Build = 0;

//            PlayerSettings.bundleVersion = MajorVersion.ToString("0") + "." +
//                                           MinorVersion.ToString("0") + "." +
//                                           Build.ToString("0");
//            PlayerSettings.Android.bundleVersionCode =
//                MajorVersion * 10000 + MinorVersion * 1000 + Build;
//        }
//    }

//    /////////////////////// Settings ///////////////////////

//    [MenuItem(AutoIncreaseMenuName, false, 1)]
//    private static void SetAutoIncrease()
//    {
//        AutoIncrease = !AutoIncrease;
//        EditorPrefs.SetBool(AutoIncreaseMenuName, AutoIncrease);
//        Debug.Log("Auto Increase : " + AutoIncrease);
//    }

//    [MenuItem(AutoIncreaseMenuName, true)]
//    private static bool SetAutoIncreaseValidate()
//    {
//        Menu.SetChecked(AutoIncreaseMenuName, AutoIncrease);
//        return true;
//    }

//    [MenuItem("Build/Check Current Version", false, 2)]
//    private static void CheckCurrentVersion()
//    {
//        CheckVersionLength(); //추가
//        Debug.Log("Build v" + PlayerSettings.bundleVersion +
//            " (" + PlayerSettings.Android.bundleVersionCode + ")");
//    }

//    [PostProcessBuild(1)]
//    public static void OnPostprocessBuild(BuildTarget target, string pathToBuiltProject)
//    {
//        CheckVersionLength(); //추가
//        if (AutoIncrease) IncreaseBuild();
//    }

//    /////////////////////// Increase ///////////////////////

//    [MenuItem("Build/Increase Major Version", false, 51)]
//    private static void IncreaseMajor()
//    {
//        string[] lines = PlayerSettings.bundleVersion.Split('.');
//        EditVersion(1, -int.Parse(lines[1]), -int.Parse(lines[2]));
//    }

//    [MenuItem("Build/Increase Minor Version", false, 52)]
//    private static void IncreaseMinor()
//    {
//        string[] lines = PlayerSettings.bundleVersion.Split('.');
//        EditVersion(0, 1, -int.Parse(lines[2]));
//    }

//    private static void IncreaseBuild()
//    {
//        EditVersion(0, 0, 1);
//    }

//    static void EditVersion(int majorIncr, int minorIncr, int buildIncr)
//    {
//        string[] lines = PlayerSettings.bundleVersion.Split('.');

//        int MajorVersion = int.Parse(lines[0]) + majorIncr;
//        int MinorVersion = int.Parse(lines[1]) + minorIncr;
//        int Build = int.Parse(lines[2]) + buildIncr;

//        PlayerSettings.bundleVersion = MajorVersion.ToString("0") + "." +
//                                       MinorVersion.ToString("0") + "." +
//                                       Build.ToString("0");
//        PlayerSettings.Android.bundleVersionCode =
//            MajorVersion * 10000 + MinorVersion * 1000 + Build;
//        CheckCurrentVersion();
//    }
//}
