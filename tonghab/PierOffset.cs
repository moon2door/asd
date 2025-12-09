using UnityEngine;

//pjh
public class PierOffset : MonoBehaviour
{
    public float pierOffset => _pierOffset;
    float _pierOffset;
    string pierName;
    void Awake()
    {
        pierName = gameObject.GetComponentInChildren<CraneParts>().pierName;
        _pierOffset = float.Parse(CsCore.Configuration.ReadConfigIni("PierAngleOffset", pierName, "0"));
        transform.localRotation = Quaternion.AngleAxis(_pierOffset, new Vector3(0, 0, 1));
    }
}
//~pjh