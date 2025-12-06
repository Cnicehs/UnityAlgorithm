using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

[ExecuteInEditMode]
public class RVOObstacleArea : MonoBehaviour
{
    [System.Serializable]
    public class ObstacleSegment
    {
        public Vector3 Point1;
        public Vector3 Point2;
        public bool IsConvex = true;
        [HideInInspector] public RVOObstacle RuntimeObstacle;
    }

    [Header("Obstacle Definition")]
    [Tooltip("Define obstacle segments manually. Each segment is a line from Point1 to Point2.")]
    public List<ObstacleSegment> Segments = new List<ObstacleSegment>();

    [Header("Auto-Generate Closed Loop")]
    [Tooltip("Define points for a closed polygon. System will auto-generate segments and link them.")]
    public List<Vector3> ClosedLoopPoints = new List<Vector3>();
    
    [Tooltip("If true, generate segments in counter-clockwise order (normals point outward).")]
    public bool CounterClockwise = true;

    [Header("Manual Segment Options")]
    [Tooltip("If true, segments list is treated as a closed loop when linking neighbors.")]
    public bool SegmentsFormClosedLoop = true;

    [Header("Visualization")]
    public Color ObstacleColor = Color.red;
    public Color NormalColor = Color.yellow;
    public bool ShowNormals = true;
    public float NormalLength = 0.5f;

    [Header("Runtime")]
    public bool AutoRegisterOnStart = true;

    private bool _isRegistered = false;

    void OnEnable()
    {
        if (Application.isPlaying && AutoRegisterOnStart)
        {
            RegisterObstacles();
        }
    }

    void OnDisable()
    {
        if (Application.isPlaying)
        {
            UnregisterObstacles();
        }
    }

    void OnDestroy()
    {
        if (Application.isPlaying)
        {
            UnregisterObstacles();
        }
    }

    public void RegisterObstacles()
    {
        if (_isRegistered) return;

        GenerateSegmentsFromClosedLoop();

        foreach (var segment in Segments)
        {
            Vector3 p1 = transform.TransformPoint(segment.Point1);
            Vector3 p2 = transform.TransformPoint(segment.Point2);

            segment.RuntimeObstacle = RVOSimulator.Instance.AddObstacle(p1, p2);
            segment.RuntimeObstacle.IsConvex = segment.IsConvex;
        }

        LinkSegments();
        _isRegistered = true;
    }

    public void UnregisterObstacles()
    {
        if (!_isRegistered) return;

        for (int i = Segments.Count - 1; i >= 0; i--)
        {
            if (Segments[i].RuntimeObstacle != null)
            {
                RVOSimulator.Instance.RemoveObstacle(Segments[i].RuntimeObstacle);
                Segments[i].RuntimeObstacle = null;
            }
        }

        _isRegistered = false;
    }

    private void GenerateSegmentsFromClosedLoop()
    {
        if (ClosedLoopPoints.Count < 3) return;

        Segments.Clear();

        for (int i = 0; i < ClosedLoopPoints.Count; i++)
        {
            int nextIndex = (i + 1) % ClosedLoopPoints.Count;
            
            Vector3 p1 = ClosedLoopPoints[i];
            Vector3 p2 = ClosedLoopPoints[nextIndex];

            if (!CounterClockwise)
            {
                (p1, p2) = (p2, p1);
            }

            ObstacleSegment segment = new ObstacleSegment
            {
                Point1 = p1,
                Point2 = p2,
                IsConvex = true
            };

            Segments.Add(segment);
        }

        DetectConvexity();
    }

    private void LinkSegments()
    {
        if (!SegmentsFormClosedLoop && ClosedLoopPoints.Count < 3)
        {
            for (int i = 0; i < Segments.Count - 1; i++)
            {
                if (Segments[i].RuntimeObstacle != null && Segments[i + 1].RuntimeObstacle != null)
                {
                    Segments[i].RuntimeObstacle.NextObstacle = Segments[i + 1].RuntimeObstacle;
                    Segments[i + 1].RuntimeObstacle.PrevObstacle = Segments[i].RuntimeObstacle;
                }
            }
        }
        else
        {
            for (int i = 0; i < Segments.Count; i++)
            {
                int nextIndex = (i + 1) % Segments.Count;
                
                if (Segments[i].RuntimeObstacle != null && Segments[nextIndex].RuntimeObstacle != null)
                {
                    Segments[i].RuntimeObstacle.NextObstacle = Segments[nextIndex].RuntimeObstacle;
                    Segments[nextIndex].RuntimeObstacle.PrevObstacle = Segments[i].RuntimeObstacle;
                }
            }
        }
    }

    [ContextMenu("Auto-Detect Convexity")]
    public void DetectConvexity()
    {
        if (Segments.Count < 3) return;

        for (int i = 0; i < Segments.Count; i++)
        {
            int prevIndex = (i - 1 + Segments.Count) % Segments.Count;
            int nextIndex = (i + 1) % Segments.Count;

            Vector3 prevP1 = Segments[prevIndex].Point1;
            Vector3 currP1 = Segments[i].Point1;
            Vector3 nextP1 = Segments[nextIndex].Point1;

            float2 p0 = new float2(prevP1.x, prevP1.z);
            float2 p1 = new float2(currP1.x, currP1.z);
            float2 p2 = new float2(nextP1.x, nextP1.z);

            float cross = RVOMath.LeftOf(p0, p1, p2);
            Segments[i].IsConvex = cross >= 0.0f;
        }
    }

    [ContextMenu("Generate Rectangle From Transform")]
    public void GenerateRectangleFromTransform()
    {
        Vector3 scale = transform.localScale;
        float halfWidth = scale.x * 0.5f;
        float halfHeight = scale.z * 0.5f;

        ClosedLoopPoints.Clear();
        ClosedLoopPoints.Add(new Vector3(-halfWidth, 0, halfHeight));   // Top-Left
        ClosedLoopPoints.Add(new Vector3(halfWidth, 0, halfHeight));    // Top-Right
        ClosedLoopPoints.Add(new Vector3(halfWidth, 0, -halfHeight));   // Bottom-Right
        ClosedLoopPoints.Add(new Vector3(-halfWidth, 0, -halfHeight));  // Bottom-Left

        CounterClockwise = true;
    }

    void OnDrawGizmos()
    {
        DrawObstacleGizmos(new Color(ObstacleColor.r, ObstacleColor.g, ObstacleColor.b, 0.3f));
    }

    void OnDrawGizmosSelected()
    {
        DrawObstacleGizmos(ObstacleColor);
    }

    private void DrawObstacleGizmos(Color color)
    {
        Gizmos.color = color;

        if (ClosedLoopPoints.Count >= 2)
        {
            for (int i = 0; i < ClosedLoopPoints.Count; i++)
            {
                Vector3 p1 = transform.TransformPoint(ClosedLoopPoints[i]);
                Vector3 p2 = transform.TransformPoint(ClosedLoopPoints[(i + 1) % ClosedLoopPoints.Count]);
                
                Gizmos.DrawLine(p1, p2);
                Gizmos.DrawSphere(p1, 0.1f);

                if (ShowNormals)
                {
                    Vector3 mid = (p1 + p2) * 0.5f;
                    Vector3 dir = (p2 - p1).normalized;
                    Vector3 normal = new Vector3(-dir.z, 0, dir.x);
                    
                    if (!CounterClockwise)
                    {
                        normal = -normal;
                    }

                    Gizmos.color = NormalColor;
                    Gizmos.DrawLine(mid, mid + normal * NormalLength);
                    Gizmos.color = color;
                }
            }
        }

        foreach (var segment in Segments)
        {
            Vector3 p1 = transform.TransformPoint(segment.Point1);
            Vector3 p2 = transform.TransformPoint(segment.Point2);

            Gizmos.color = segment.IsConvex ? color : Color.magenta;
            Gizmos.DrawLine(p1, p2);
            Gizmos.DrawSphere(p1, 0.08f);

            if (ShowNormals)
            {
                Vector3 mid = (p1 + p2) * 0.5f;
                Vector3 dir = (p2 - p1).normalized;
                Vector3 normal = new Vector3(-dir.z, 0, dir.x);

                Gizmos.color = segment.IsConvex ? NormalColor : Color.cyan;
                Gizmos.DrawLine(mid, mid + normal * NormalLength);
            }
        }
    }

#if UNITY_EDITOR
    void OnValidate()
    {
        if (ClosedLoopPoints.Count >= 3)
        {
            GenerateSegmentsFromClosedLoop();
        }
    }
#endif
}
