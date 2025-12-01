using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class ZCenter : MonoBehaviour
{
    public bool zCheck = false;
    void OnDrawGizmos()
    {
        if(zCheck)
        {
            MeshFilter mf = GetComponent<MeshFilter>();
            if (mf == null) return;

            Mesh mesh = mf.sharedMesh;
            if (mesh == null) return;

            Vector3[] verts = mesh.vertices;
            if (verts.Length == 0) return;

            // z축 기준 최대값 찾기
            float maxZ = float.MinValue;
            foreach (var v in verts)
            {
                if (v.z > maxZ) maxZ = v.z;
            }

            // 최대 z에 해당하는 모든 점 찾기
            List<Vector3> edgeVerts = new List<Vector3>();
            foreach (var v in verts)
            {
                if (Mathf.Approximately(v.z, maxZ))
                {
                    edgeVerts.Add(transform.TransformPoint(v));
                }
            }

            // 점 찍기 (크기 1짜리 구체)
            Gizmos.color = Color.red;
            foreach (var p in edgeVerts)
            {
                Gizmos.DrawSphere(p, 0.1f);
            }
        }
        else
        {
            MeshFilter mf = GetComponent<MeshFilter>();
            if (mf == null) return;

            Mesh mesh = mf.sharedMesh;
            if (mesh == null) return;

            Vector3[] verts = mesh.vertices;
            if (verts.Length == 0) return;

            // z축 기준 최대값 찾기
            float minZ = float.MaxValue;
            foreach (var v in verts)
            {
                if (v.z < minZ) minZ = v.z;
            }

            // 최대 z에 해당하는 모든 점 찾기
            List<Vector3> edgeVerts = new List<Vector3>();
            foreach (var v in verts)
            {
                if (Mathf.Approximately(v.z, minZ))
                {
                    edgeVerts.Add(transform.TransformPoint(v));
                }
            }

            // 점 찍기 (크기 1짜리 구체)
            Gizmos.color = Color.red;
            foreach (var p in edgeVerts)
            {
                Gizmos.DrawSphere(p, 0.1f);
            }
        }
    }
}
