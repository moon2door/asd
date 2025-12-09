using UnityEditor;
using UnityEngine;

#if UNITY_EDITOR
//pjh
[CustomEditor(typeof(CraneParts))]
public class CranePartsEditor : Editor
{
    SerializedProperty craneTypeProp;
    SerializedProperty craneBodyProp;
    SerializedProperty operationRoomProp;
    SerializedProperty trollyProp;
    SerializedProperty hook1HighProp;
    SerializedProperty hook1LowProp;
    SerializedProperty hook2HighProp;
    SerializedProperty hook2LowProp;
    SerializedProperty hook3HighProp;
    SerializedProperty hook3LowProp;

    private void OnEnable()
    {
        // SerializedProperty에 각 필드를 매핑
        craneTypeProp = serializedObject.FindProperty("craneType");
        craneBodyProp = serializedObject.FindProperty("craneBody");
        operationRoomProp = serializedObject.FindProperty("operationRoom");
        trollyProp = serializedObject.FindProperty("trolly");
        hook1HighProp = serializedObject.FindProperty("hook1High");
        hook1LowProp = serializedObject.FindProperty("hook1Low");
        hook2HighProp = serializedObject.FindProperty("hook2High");
        hook2LowProp = serializedObject.FindProperty("hook2Low");
        hook3HighProp = serializedObject.FindProperty("hook3High");
        hook3LowProp = serializedObject.FindProperty("hook3Low");
    }

    public override void OnInspectorGUI()
    {
        // serializedObject 업데이트
        serializedObject.Update();

        // 기본 필드들은 DrawDefaultInspector()로 출력하지 않고 수동으로 출력
        EditorGUILayout.PropertyField(serializedObject.FindProperty("isWork"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("craneIndex"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("pierCode"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("craneCode"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("pierName"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("craneName"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("sensorCount"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("pointCloudTransform"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("refPosition"));

        EditorGUILayout.PropertyField(serializedObject.FindProperty("useReverseRotateCrane"));

        EditorGUILayout.PropertyField(serializedObject.FindProperty("pierDir"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("cranePositionOffset"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("bodyAngleOffset"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("jibAngleOffset"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("clampMin"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("clampMax"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("clampValue"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("pointCloudPositionOffset"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("pointCloudRotationOffset"));

        EditorGUILayout.PropertyField(serializedObject.FindProperty("craneJib"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("craneTower"));

        EditorGUILayout.PropertyField(serializedObject.FindProperty("debugMode"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("testGPS1"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("testGPS2"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("t_azimuth"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("t_highAngle"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("currentGPS"));

        // enum 필드 출력
        EditorGUILayout.PropertyField(craneTypeProp);

        // LLC 선택 시
        if (craneTypeProp.enumValueIndex != (int)CraneParts.CraneType.GC)
        {
            EditorGUILayout.PropertyField(craneBodyProp, new GUIContent("Crane Body"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("useReverseRotator"));
        }
        // GC 선택 시
        else if (craneTypeProp.enumValueIndex == (int)CraneParts.CraneType.GC)
        {
            EditorGUILayout.PropertyField(operationRoomProp, new GUIContent("Operation Room"));
            EditorGUILayout.PropertyField(trollyProp, new GUIContent("Trolly"));
            EditorGUILayout.PropertyField(hook1HighProp, new GUIContent("Hook 1 High"));
            EditorGUILayout.PropertyField(hook1LowProp, new GUIContent("Hook 1 Low"));
            EditorGUILayout.PropertyField(hook2HighProp, new GUIContent("Hook 2 High"));
            EditorGUILayout.PropertyField(hook2LowProp, new GUIContent("Hook 2 Low"));
            EditorGUILayout.PropertyField(hook3HighProp, new GUIContent("Hook 3 High"));
            EditorGUILayout.PropertyField(hook3LowProp, new GUIContent("Hook 3 Low"));
        }
        
        EditorGUILayout.PropertyField(serializedObject.FindProperty("exceptionTower"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("exceptionJIB"));
        // 변경 사항을 적용
        serializedObject.ApplyModifiedProperties();
    }
}
#endif
//~pjh