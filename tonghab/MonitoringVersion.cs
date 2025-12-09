using UnityEngine;
using UnityEngine.UI;
using UnityEditor;

public class MonitoringVersion : MonoBehaviour
{
    Text versionText;

    private void Start() {
        versionText = GetComponent<Text>();

        versionText.text = "Version - " + Application.version;
        
    }
    
}
