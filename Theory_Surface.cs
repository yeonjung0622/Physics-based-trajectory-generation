using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.Threading.Tasks;

public class Theory_Surface : MonoBehaviour
{
    private Collider surfaceCollider;
    //인스펙터 접근 불가 변수 (private)
    private MeshFilter meshFilterComponent;
    private Mesh mesh;
    private Vector3[] vertices;
    private Color[] colors;
    private float[] waterAmounts;

    //레이 회전 반영 관련 변수
    private Coroutine calculationCoroutine;
    private Quaternion lastRotation;
    public Transform raycastSource;

    //2단계에서 필요한 변수들
    private Vector3[] normals;
    public List<Vector3> dripPoints = new List<Vector3>();

    //3단계에서 필요한  변수
    public float maxIcicleLength = 40.0f;
    public Material ice;
    private List<LineRenderer> icicleRenderers = new List<LineRenderer>();

    private List<GameObject> generatedIcicleSegments = new List<GameObject>();
    private Material lineMaterial;

    //각 버텍스 이웃 정보를 저장할 딕셔너리
    private Dictionary<int, List<int>> vertexNeighbors;

    //인스펙터 설정 public 변수들
    public float colorChangeRadius = 0.1f;
    public Color minWaterColor = Color.red;
    public Color maxWaterColor = Color.blue;
    public float maxWaterAmount = 1.0f;
    public float waterAddAmount = 1.0f;
    public float waterSupplyThreshold = 0.001f;
    public int verticesPerFrame = 500;

    public Vector3 currentGravityDirection = Vector3.down;
    //물 정리 
    private float[] waterSupply;
    private float[] waterCoeff;

    //2단계에서 필요한 변수들
    public float dripLimitAngle = 75.0f;
    public int numberOfIcicles = 15;
    public List<int> gizmoVertexIndices;
    public float gizmoSize = 0.3f;


    [Header("Icicle Growth Parameters")]
    public float segmentLength = 2.0f;
    public LayerMask collisionLayers;

    //4단계 필요 변수들 (IcicleProfile)
    [Header("Icicle Profile parameters")]
    public float rippleFrequency = 20.0f; //fs
    public float taper = 0.1f; //t
    public float rippleAmplitude = 0.1f; //as
    public float tipRadius = 0.15f;

    //noise 함수
    [Header("Noise")]
    public float positionNoiseAmount = 0.5f;

    [Header("Base of Icicles")]
    public float baseSpreadDistance = 1.0f;
    public int baseMetaballCount = 100;
    public float minRadius = 0.01f;

    //4단계 필요 변수들 (Glaze Ice)
    [Header("Glaze Ice")]
    public int ngi = 5000; //사용자가 설정할 메타볼 개수 
    public float scaling = 0.03f; // s <-빙막 스케일링 값 
    public float lifeTime = 0.5f;  // lt <- 물 방울이 살아남는 생존 시간 (0~1 사이의 값) 
    public float minGI = 0;

    //블루프린트 노이즈 샘플링 필요 변수들 
    private float[] triangleAreas;
    private float totalArea;
    private float[] cumulativeAreas;
    private bool isComputing = false;

    public struct IceCandidate
    {
        public int index;
        public Vector3 localPos;
        public Vector3 normal;
        public float waterAmount;
        public float radius;
        public float weight;
        public bool eliminated;
    }
    public float dropletRadiusMm = 2.44f;
    [Header("Physical Environment")]
    [Tooltip("주변 기온 (섭씨)")]
    public float Temperature = -9.0f;

    [Header("Wind Settings")]
    public Vector3 windVector = new Vector3(5f, 0f, 0f); // X, Z축으로 바람 세기 조절

    [Tooltip("고드름 성장 속도 (mm/s)")]
    public float v_speed_mm = 3.0f;
    private Vector3 lastWindVector;
    private GameObject glazeIceContainer;

    [Header("Real-time Growth Settings")]
    public float growthInterval = 0.05f; // 마디 하나가 생성되는 시간 간격 (초)

    [Header("Physical Drip Settings")]
    [Tooltip("중력/접착력 비율 임계값: 높을수록 물이 더 무거워져야 떨어집니다.")]
    public float dripForceRatioThreshold = 1.5f;

    [Tooltip("표면장력 계수: 물의 기본 접착력을 결정합니다.")]
    public float surfaceTension = 0.072f;

    [Tooltip("모서리 접착력 감소율: 높을수록 뾰족한 곳에서 고드름이 더 잘 생깁니다.")]
    public float edgeSensitivity = 2.0f;

    [Tooltip("드립 포인트 간 최소 거리: 고드름이 너무 뭉치는 것을 방지합니다.")]
    public float minDripDistance = 0.1f;

    [Header("Wind Normal Debug")]
    public bool debugWindNormal = true;
    [Range(3, 24)] public int debugSide = 6; 
    public float debugNormalLength = 0.04f;
    public float debugAxisLength = 0.05f;
    public float debugDuration = 2.0f;
    public int debugEveryNthBall = 6;

    //추가 리플 메타볼 생성 변수 추가
    [Header("Ripple of metaball")]
    public bool enableRippleMetaball = true;
    public int rippleSegmentInterval = 2;
    [Range(3, 12)] public int rippleSegmentCount = 6;
    public float windwardRippleRatio = 0.08f;
    public float leewardRippleRatio = 0.75f;
    [Range(0f, 1f)] public float rippleEmbedRatio = 0.05f;
    [Range(0f, 1f)] public float rippleTipFadeStart = 0.8f;

    [Tooltip("이 풍속에 가까워질수록 풍상/풍하 리플 차이가 최대로 커짐")]
    public float rippleReferenceWindSpeed = 10.0f;

    [Tooltip("바람 영향이 약할 때 기본 리플 크기")]
    public float rippleNeutralRatio = 0.05f;

    [Header("Cross-section Wind Asymmetry")]
    [Range(0f, 0.3f)] public float windwardBodyShrink = 0.10f;
    [Range(0f, 0.5f)] public float leewardBodyExpand = 0.25f;
    [Range(0f, 0.35f)] public float leewardCenterShift = 0.18f;

    [Range(0f, 1f)] public float rippleRootFadeStart = 0.12f;
    [Range(0f, 1f)] public float rippleRootFadeEnd = 0.28f;




    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("Start() 함수 시작!");
        meshFilterComponent = GetComponent<MeshFilter>();

        //1.메시 필터가 없는 경우
        if (meshFilterComponent == null)
        {
            Debug.LogWarning("MeshFilter가 오브젝트에 없습니다.", this);
            enabled = false;
            return;
        }
        Debug.Log("Start(): Step 1 - MeshFilter 컴포넌트 찾기 성공.", this);

        mesh = meshFilterComponent.mesh;
        surfaceCollider = GetComponent<MeshCollider>();

        //2. 메시 데이터가 없는 경우
        if (mesh == null)
        {
            Debug.LogWarning("메시 데이터가 없습니다.", this);
            enabled = false;
            return;
        }
        Debug.Log("Start(): Step 2 - Mesh 객체 가져오기 성공.", this);
        //버텍스 데이터 초기화
        vertices = mesh.vertices;

        Debug.Log("Start(): Step 3 - BuildVertexNeighbors() 호출 직전.", this);

        //color 배열 초기화 / 기존 컬러 없으면 흰색으로 초기화 

        if (mesh.colors.Length == vertices.Length)
        {
            colors = mesh.colors;
        }

        else
        {
            colors = new Color[vertices.Length];
            for (int i = 0; i < colors.Length; i++)
            {
                colors[i] = Color.white;
            }
        }

        waterSupply = new float[vertices.Length];
        waterCoeff = new float[vertices.Length];

        for (int i = 0; i < vertices.Length; i++)
        {
            waterSupply[i] = 0f;
            waterCoeff[i] = 0f;
        }

        if (gizmoVertexIndices == null) gizmoVertexIndices = new List<int>();
        //이웃 버텍스 정보를 빌드하는 함수 호출
        BuildVertexNeighbors();
        Debug.Log("Start(): Step 4 - BuildVertexNeighbors() 호출 완료.", this);

        vertices = mesh.vertices;
        normals = mesh.normals;

        mesh.colors = colors;

        Debug.Log("Start() 초기화 완료.", this);

        lineMaterial = new Material(Shader.Find("Sprites/Default"));
    }

    // Update is called once per frame
    void Update()
    {
        if (raycastSource == null) return;

        float angleDiff = Quaternion.Angle(raycastSource.rotation, lastRotation);
        if (angleDiff <= 1.0f) return;

        if (calculationCoroutine != null) StopCoroutine(calculationCoroutine);
        calculationCoroutine = StartCoroutine(WaterCoefficientCalculateCoroutine());
        lastRotation = raycastSource.rotation;

    }

    public void WaterSupplyVertex(Vector3 worldHitPoint, Vector3 rayDirection)
    {
        if (mesh == null) return;

        this.currentGravityDirection = rayDirection;

        Vector3 localHitPoint = transform.InverseTransformPoint(worldHitPoint);

        for (int i = 0; i < vertices.Length; i++)
        {
            float distance = Vector3.Distance(vertices[i], localHitPoint);
            if (distance < colorChangeRadius)
            {
                //waterAmounts[i] = Mathf.Max(waterAmounts[i], waterAddAmount);
                waterSupply[i] = Mathf.Max(waterSupply[i], waterAddAmount);

            }
        }
        if (calculationCoroutine != null) StopCoroutine(calculationCoroutine);
        calculationCoroutine = StartCoroutine(WaterCoefficientCalculateCoroutine());

    }

    //이웃 정점 찾는 함수(start에서 호출)
    private void BuildVertexNeighbors()
    {
        vertexNeighbors = new Dictionary<int, List<int>>();

        int[] triangles = mesh.triangles;

        for (int i = 0; i < vertices.Length; i++)
        {
            vertexNeighbors.Add(i, new List<int>());
        }

        //삼각형은 3개의 버텍스 인덱스로 구성, i를 3씩 증가시키며 모든 삼각형을 처리
        for (int i = 0; i < triangles.Length; i += 3)
        {
            int v1 = triangles[i];
            int v2 = triangles[i + 1];
            int v3 = triangles[i + 2];

            //각 버텍스에 대해 나머지 두 버텍스를 이웃으로 추가 
            AddUniqueNeighbor(v1, v2);
            AddUniqueNeighbor(v1, v3);
            AddUniqueNeighbor(v2, v1);
            AddUniqueNeighbor(v2, v3);
            AddUniqueNeighbor(v3, v1);
            AddUniqueNeighbor(v3, v2);
        }

        Debug.Log("이웃 버텍스 정보 빌드 완료. 총 버텍스 수 " + vertices.Length);

    }

    //메쉬 영역 계산 
    private void CalculateMeshAreas()
    {
        Mesh tragetMesh = meshFilterComponent.sharedMesh;
        int[] triangles = mesh.triangles;
        Vector3[] verts = mesh.vertices;

        int triCount = triangles.Length / 3;
        triangleAreas = new float[triCount];
        cumulativeAreas = new float[triCount];
        totalArea = 0f;

        for (int i = 0; i < triCount; i++)
        {
            Vector3 v0 = verts[triangles[i * 3]];
            Vector3 v1 = verts[triangles[i * 3 + 1]];
            Vector3 v2 = verts[triangles[i * 3 + 2]];

            float area = Vector3.Cross(v1 - v0, v2 - v0).magnitude * 0.5f;
            totalArea += area;
            triangleAreas[i] = area;
            cumulativeAreas[i] = totalArea;
        }

    }



    //이웃 리스트에 중복 없이 버텍스를 추가하는 함수
    private void AddUniqueNeighbor(int vertexIndex, int neighborIndex)
    {
        if (!vertexNeighbors.ContainsKey(vertexIndex))
        {
            vertexNeighbors.Add(vertexIndex, new List<int>());
        }
        if (!vertexNeighbors[vertexIndex].Contains(neighborIndex))
        {
            vertexNeighbors[vertexIndex].Add(neighborIndex);
        }
    }


    //물 계수 함수 알고리즘
    private IEnumerator WaterCoefficientCalculateCoroutine()
    {
        // 중력 방향(레이 컨트롤러 회전 반영)
        Vector3 worldG = raycastSource ? raycastSource.TransformDirection(Vector3.down) : Vector3.down;
        Vector3 g = transform.InverseTransformDirection(worldG).normalized; // vertices와 동일 좌표계

        // 결과 배열(계수) : 계산 중 원본을 덮어쓰지 않음
        float[] newCoeff = new float[vertices.Length];

        int maxSearchIterations = vertices.Length * 2;
        int processed = 0;


        float runningMax = 1e-6f;

        for (int v_index = 0; v_index < vertices.Length; v_index++)
        {
            int c = v_index;
            Vector3 cPos = vertices[c];

            float wc = 0f;

            HashSet<int> visited = new HashSet<int>();
            visited.Add(c);

            int iter = 0;

            while (true)
            {
                iter++;
                if (iter > maxSearchIterations)
                {
                    Debug.LogWarning($"WaterCoeff: vertex {v_index} 탐색 루프 과다. 강제 종료");
                    break;
                }

                float minP = float.MaxValue;
                int nMin = -1;

                if (vertexNeighbors.TryGetValue(c, out var neighbors))
                {
                    foreach (int n in neighbors)
                    {
                        if (visited.Contains(n)) continue;

                        Vector3 nPos = vertices[n];
                        Vector3 cn = (nPos - cPos).normalized;

                        float p = Vector3.Dot(cn, g);   // 논문식 p값
                        if (p < minP)
                        {
                            minP = p;
                            nMin = n;
                        }
                    }
                }

                if (nMin == -1) break;


                bool cIsSupply = (waterSupply[c] > waterSupplyThreshold);
                bool nIsSupply = (waterSupply[nMin] > waterSupplyThreshold);

                float d = Vector3.Distance(cPos, vertices[nMin]);
                float result = d * (-minP);

                if (cIsSupply != nIsSupply) result *= 0.5f;

                wc += result;

                // 다음으로 이동
                c = nMin;
                cPos = vertices[c];
                visited.Add(c);
            }

            newCoeff[v_index] = wc;
            runningMax = Mathf.Max(runningMax, wc);

            processed++;

            // 프레임 분할 업데이트(색 미리보기)
            if (processed % verticesPerFrame == 0)
            {
                // 임시 색 업데이트: 지금까지 계산된 값 기준
                for (int i = 0; i < vertices.Length; i++)
                {
                    float normalized = Mathf.Clamp01(newCoeff[i] / runningMax);
                    colors[i] = Color.Lerp(minWaterColor, maxWaterColor, normalized);
                }
                mesh.colors = colors;

                yield return null;
            }
        }

        // 최종 결과를 waterCoeff에 저장
        System.Array.Copy(newCoeff, waterCoeff, vertices.Length);

        // 정규화 기준(maxWaterAmount)은 “waterCoeff 최대값”으로 1번만 갱신
        float maxCoeff = 0f;
        for (int i = 0; i < waterCoeff.Length; i++)
            if (waterCoeff[i] > maxCoeff) maxCoeff = waterCoeff[i];

        maxWaterAmount = Mathf.Max(1e-6f, maxCoeff);


        for (int i = 0; i < vertices.Length; i++)
        {
            float normalized = Mathf.Clamp01(waterCoeff[i] / maxWaterAmount);
            colors[i] = Color.Lerp(minWaterColor, maxWaterColor, normalized);
        }
        mesh.colors = colors;

        Debug.Log("물 계수 계산 완료(분리 구조).", this);

        DripPointsIdentification();

        GlazeIce();
    }

    //추가 
    private float ComputeDripForceRatio(int prev, int curr, Vector3 gLocal)
    {
        float mass = waterCoeff[curr];
        float gravityComponent = Vector3.Dot(normals[curr], gLocal);
        float F_grav = mass * Mathf.Max(0.1f, gravityComponent);

        float curvature = 1f - Vector3.Dot(normals[prev], normals[curr]);

        float adhesiveScale = 1f / (1f + curvature * edgeSensitivity);
        float F_adhesive = surfaceTension * adhesiveScale;

        return F_grav / Mathf.Max(1e-6f, F_adhesive);
    }

    private int FindPhysicalDripPoint(int startIdx, Vector3 gLocal)
    {
        int current = startIdx;
        int prev = startIdx;
        int steps = 0;

        while (steps++ < 512)
        {
            if (!vertexNeighbors.TryGetValue(current, out var nbs) || nbs.Count == 0)
                break;

            int next = -1;
            float maxDrop = 0f;
            float hCur = Vector3.Dot(vertices[current], -gLocal);

            // 가장 낮은 이웃 찾기
            foreach (int n in nbs)
            {
                float hN = Vector3.Dot(vertices[n], -gLocal);
                float dH = hCur - hN;
                if (dH > maxDrop)
                {
                    maxDrop = dH;
                    next = n;
                }
            }

            if (next == -1) break; // 국소 최저점 도달

            float ratio = ComputeDripForceRatio(current, next, gLocal);
            if (ratio > dripForceRatioThreshold)
            {
                return next; // 중력이 표면 장력을 이겼으므로 여기서 낙하 시작
            }

            current = next;
        }
        return current;
    }

    //2단계: 고드름 드립 포인트 찾기
    public void DripPointsIdentification()
    {
        dripPoints.Clear();
        gizmoVertexIndices.Clear();

        Vector3 worldG = raycastSource ? raycastSource.TransformDirection(Vector3.down) : Vector3.down;
        Vector3 g = transform.InverseTransformDirection(worldG).normalized;

        float minHeightAlongGravity = float.MaxValue;
        for (int i = 0; i < vertices.Length; i++)
        {
            float h = Vector3.Dot(vertices[i], -g);  // 중력 반대 방향의 높이
            if (h < minHeightAlongGravity) minHeightAlongGravity = h;
        }

        float gravityBasedBottomThreshold = minHeightAlongGravity + 0.05f;

        float angleThreshold = Mathf.Cos(dripLimitAngle * Mathf.Deg2Rad);

        Debug.Log($"[DripPoints] AngleThreshold(cos): {angleThreshold:F3}, HeightThreshold: {gravityBasedBottomThreshold:F3}");

        List<int> seedVertices = new List<int>();

        for (int i = 0; i < vertices.Length; i++)
        {
            float water = waterCoeff[i];

            // 1. 물이 충분히 있는가?
            if (water < 0.01f) continue;

            // 2. 중력 방향 기준으로 너무 아래쪽인가?
            float heightAlongGravity = Vector3.Dot(vertices[i], -g);
            if (heightAlongGravity < gravityBasedBottomThreshold)
                continue;

            // 3. 표면이 충분히 아래를 향하는가? (중력 방향과 일치)
            float downDot = Vector3.Dot(normals[i], g);
            if (downDot < angleThreshold)
                continue;

            seedVertices.Add(i);
        }

        Debug.Log($"[DripPoints] SeedVertices found: {seedVertices.Count}");

        if (seedVertices.Count == 0)
        {
            Debug.LogWarning("[DripPoints] 적합한 드립 후보 정점 없음. " +
                             $"waterThreshold=0.01, heightThreshold={gravityBasedBottomThreshold:F3}, " +
                             $"angleThreshold={dripLimitAngle}도");
            return;
        }

        // 랜덤 셔플
        for (int i = 0; i < seedVertices.Count; i++)
        {
            int swap = Random.Range(i, seedVertices.Count);
            (seedVertices[i], seedVertices[swap]) = (seedVertices[swap], seedVertices[i]);
        }

        // 물리적 흐름 추적 및 드립 포인트 확정
        HashSet<int> usedIndices = new HashSet<int>();
        List<int> finalDripIndices = new List<int>();

        for (int i = 0; i < seedVertices.Count && finalDripIndices.Count < numberOfIcicles; i++)
        {
            int seedIdx = seedVertices[i];
            int dripIdx = FindPhysicalDripPoint(seedIdx, g);

            // 이미 사용된 정점인가?
            if (usedIndices.Contains(dripIdx))
                continue;

            // 거리 기반 중복 방지
            bool tooClose = false;
            foreach (int existingIdx in finalDripIndices)
            {
                float dist = Vector3.Distance(vertices[dripIdx], vertices[existingIdx]);
                if (dist < minDripDistance)
                {
                    tooClose = true;
                    break;
                }
            }

            if (!tooClose)
            {
                usedIndices.Add(dripIdx);
                finalDripIndices.Add(dripIdx);
                dripPoints.Add(vertices[dripIdx]);
                gizmoVertexIndices.Add(dripIdx);
            }
        }

        Debug.Log($"[DripPoints] Final drip points: {finalDripIndices.Count}/{numberOfIcicles}");

        lastWindVector = windVector;
        StartCoroutine(GenerateIcicles());
    }


    void OnDrawGizmos()
    {
        // 필요한 데이터(물 양, 정점, 시각화할 인덱스)가 모두 있는지 확인
        if (waterCoeff != null && vertices != null && gizmoVertexIndices != null)
        {


            // 시각화할 정점 인덱스 리스트를 순회
            foreach (int i in gizmoVertexIndices)
            {
                // 배열 인덱스 범위를 벗어나지 않도록 검사
                if (i >= 0 && i < vertices.Length && i < waterCoeff.Length)
                {
                    if (waterCoeff[i] / maxWaterAmount >= 0.5f)
                    {
                        Gizmos.color = maxWaterColor; // 물이 많을 때는 파란색 등으로
                    }
                    else
                    {
                        Gizmos.color = minWaterColor; // 물이 적을 때는 빨간색 등으로
                    }

                    // 현재 정점의 로컬 위치를 월드 위치로 변환
                    Vector3 worldPosition = transform.TransformPoint(vertices[i]);

                    // 해당 위치에 색상이 지정된 구체 기즈모를 그림
                    Gizmos.DrawSphere(worldPosition, gizmoSize);

                }
            }
        }


    }

    private IEnumerator GenerateIcicles()
    {
        // 고드름 초기화 
        foreach (LineRenderer lr in icicleRenderers)
        {
            if (lr != null && lr.gameObject != null)
            {
                Destroy(lr.gameObject);
            }
        }
        icicleRenderers.Clear();

        foreach (GameObject segment in generatedIcicleSegments)
        {
            if (segment != null)
            {
                Destroy(segment);
            }
        }
        generatedIcicleSegments.Clear();

        // 스케일 감지
        Vector3 currentScale = transform.lossyScale;
        float avgScale = (currentScale.x + currentScale.y + currentScale.z) / 3.0f;

        // 드립포인트마다 고드름 생성 
        for (int i = 0; i < numberOfIcicles && i < dripPoints.Count; i++)
        {
            StartCoroutine(GrowSingleIcicleRealtime(i, avgScale));
            yield return null;
        }

        Debug.Log($"총 {numberOfIcicles}개의 고드름 생성 완료");
        isComputing = false;
    }

    // 개별 고드름이 실시간으로 자라나는 코루틴
    private IEnumerator GrowSingleIcicleRealtime(int icicleIndex, float avgScale)
    {
        Transform meshTr = this.transform;

        int vIdx = gizmoVertexIndices[icicleIndex];
        Vector3 worldDripPoint = meshTr.TransformPoint(vertices[vIdx]);
        Vector3 worldNormal = meshTr.TransformDirection(normals[vIdx]).normalized;
        Vector3 currentWorldPos = worldDripPoint + worldNormal * (0.01f * avgScale);

        // 1) LineRenderer 세팅 (회전은 따라가되, 스케일은 중화)
        GameObject trajectoryHelper = new GameObject($"IcicleLine_{icicleIndex}");
        trajectoryHelper.transform.SetParent(meshTr, false);
        trajectoryHelper.transform.localPosition = Vector3.zero;
        trajectoryHelper.transform.localRotation = Quaternion.identity;

        // (선택) 부모가 거대한 스케일이면 라인 두께/좌표 왜곡 방지용
        Vector3 ls = meshTr.lossyScale;
        trajectoryHelper.transform.localScale = new Vector3(
            (ls.x != 0f) ? 1f / ls.x : 1f,
            (ls.y != 0f) ? 1f / ls.y : 1f,
            (ls.z != 0f) ? 1f / ls.z : 1f
        );

        LineRenderer lr = trajectoryHelper.AddComponent<LineRenderer>();
        icicleRenderers.Add(lr);


        lr.useWorldSpace = false;
        lr.startWidth = 0.02f * avgScale;
        lr.endWidth = 0.005f * avgScale;
        lr.material = lineMaterial;

        // 로컬 좌표 리스트
        List<Vector3> localPoints = new List<Vector3>();
        Vector3 initialLocalPos = trajectoryHelper.transform.InverseTransformPoint(currentWorldPos);
        localPoints.Add(initialLocalPos);

        lr.positionCount = 1;
        lr.SetPosition(0, initialLocalPos);

        // 2) 성장 상태 변수
        float current_tan_phi = 0.0f;
        float accumulatedLength = 0f;

        float waterValue = Mathf.Clamp01(waterCoeff[vIdx] / maxWaterAmount);
        //float maxLength = maxIcicleLength * waterValue;

        //수분량이 많을수록 (1에 가까울수록) 최대 길이 감소
        float w = Mathf.Clamp01(waterValue);
        float maxLength = maxIcicleLength / (1.0f + 3.0f * w);

        bool isOnSurface = false;
        Vector3 currentNormal = worldNormal;

        float lift = 0.01f * avgScale;   // 표면에서 살짝 띄우기
        float snapUp = 0.10f * avgScale;   // 스냅 시작 오프셋
        float snapDist = 0.30f * avgScale;   // 스냅 거리

        //float currentAsym = 0f;
        float simDt = growthInterval;

        float Aw = 0f;
        // 3) 실시간 성장 루프
        while (accumulatedLength < maxLength)
        {
            // 회전 중에도 정확히 붙기: 마지막 로컬 점의 "현재 월드" 위치 재계산
            currentWorldPos = trajectoryHelper.transform.TransformPoint(localPoints[localPoints.Count - 1]);

            // 바람(월드 고정, 수평 성분만)
            Vector3 windH = new Vector3(windVector.x, 0f, windVector.z);
            float currentWindSpeed = windH.magnitude;
            Vector3 windDirectionHorizontal = (windH.sqrMagnitude > 1e-6f) ? windH.normalized : Vector3.zero;

            //이론 성장항
            float rMeters = dropletRadiusMm / 1000f;
            float eta = IciclePhysics.CalculateEta(Temperature);
            float Re = IciclePhysics.CalculateReynolds(currentWindSpeed, rMeters, eta);
            float cd = IciclePhysics.CalculateCd(Re);
            float tan_theory = IciclePhysics.CalculateTanTheory(currentWindSpeed, rMeters, cd);
            float beta = IciclePhysics.ComputeBeta(v_speed_mm, segmentLength);

            //비대칭 성장항
            Vector3 localWindDir = meshTr.InverseTransformDirection(windDirectionHorizontal).normalized;
            float kappa = IciclePhysics.ComputeKappa(rMeters);
            Aw = IciclePhysics.CalculateAw(vIdx, localWindDir, vertices, waterCoeff, vertexNeighbors);
            float gamma = IciclePhysics.ComputeGamma(Temperature, waterValue);

            float I_u = IciclePhysics.CalculateIU(currentWindSpeed);
            float dAsym = IciclePhysics.ComputeAsymmetricGrowth(kappa, I_u, Aw, gamma) * simDt;
            float tanBefore = current_tan_phi;
            float dTheory = beta * (tan_theory - tanBefore) * simDt;
            float tanAfter = tanBefore + dTheory + dAsym;

            tanAfter = Mathf.Clamp(tanAfter, 0f, Mathf.Tan(75f * Mathf.Deg2Rad));
            current_tan_phi = tanAfter;


            if (tanAfter >= Mathf.Tan(75f * Mathf.Deg2Rad))
            {
                Debug.LogWarning("[Clamp Warning] 각도가 75도 제한에 도달했습니다!");
            }

            // 다음 위치 기본값(월드)
            Vector3 stepDirection = (Vector3.down + windDirectionHorizontal * current_tan_phi).normalized;
            float stepLen = segmentLength;
            Vector3 nextWorldPos = currentWorldPos + stepDirection * stepLen;

            RaycastHit hit;
            float windSpeed = windVector.magnitude;
            float currentRadius = IcicleProfile(accumulatedLength, maxLength, windSpeed);
            float dynamicLift = Mathf.Max(lift, currentRadius * 0.8f);
            //표면 충돌 및 슬라이딩
            // A) 공중 상태: 표면 찾기
            if (!isOnSurface)
            {
                // 표면 탐색은 stepDir보다 down이 안정적 (원하면 stepDir로 바꿔도 됨)
                if (Physics.Raycast(currentWorldPos, stepDirection, out hit, stepLen * 1.5f, collisionLayers))
                {
                    if (surfaceCollider == null || hit.collider == surfaceCollider)
                    {
                        nextWorldPos = hit.point + hit.normal * dynamicLift;
                        currentNormal = hit.normal;
                        isOnSurface = true;
                    }
                    // else: 바닥/다른 콜라이더 맞춘 것 -> 무시하고 공중 진행 유지
                }
            }
            // B) 표면 상태: 슬라이딩 + 재스냅
            else
            {

                if (IsDripRegion(currentWorldPos, currentNormal))
                {
                    isOnSurface = false;
                }
                else
                {
                    // 1) 현재 위치를 표면에 스냅
                    RaycastHit snapHit;
                    if (Physics.Raycast(currentWorldPos + currentNormal * snapUp,
                                        -currentNormal,
                                        out snapHit,
                                        snapDist,
                                        collisionLayers)
                        && (surfaceCollider == null || snapHit.collider == surfaceCollider))
                    {
                        currentNormal = snapHit.normal;
                        Vector3 snappedPos = snapHit.point + currentNormal * dynamicLift;

                        // 2) 표면 접선 방향으로 이동
                        Vector3 slideDir = Vector3.ProjectOnPlane(stepDirection, currentNormal);
                        if (slideDir.sqrMagnitude < 1e-8f)
                        {
                            isOnSurface = false;
                        }
                        else
                        {
                            slideDir.Normalize();
                            Vector3 targetPos = snappedPos + slideDir * stepLen;

                            // 3) 이동한 지점에서 다시 스냅 (표면 유지)
                            if (Physics.Raycast(targetPos + currentNormal * snapUp,
                                                -currentNormal,
                                                out hit,
                                                snapDist,
                                                collisionLayers)
                                && (surfaceCollider == null || hit.collider == surfaceCollider))
                            {
                                nextWorldPos = hit.point + hit.normal * dynamicLift;
                                currentNormal = hit.normal;
                            }
                            else
                            {

                                if (Physics.Linecast(snappedPos, targetPos, out hit, collisionLayers)
                                && (surfaceCollider == null || hit.collider == surfaceCollider))
                                {
                                    // 가다가 벽에 막히면 그 벽에 붙음
                                    nextWorldPos = hit.point + hit.normal * dynamicLift;
                                    currentNormal = hit.normal;
                                }
                                else if (Physics.Raycast(targetPos, Vector3.down, out hit, stepLen * 2.0f, collisionLayers)
                                    && (surfaceCollider == null || hit.collider == surfaceCollider))
                                {
                                    nextWorldPos = hit.point + hit.normal * dynamicLift;
                                    currentNormal = hit.normal;
                                    isOnSurface = true;
                                }
                                else
                                {
                                    isOnSurface = false;
                                    nextWorldPos = targetPos;
                                }
                            }
                        }
                    }
                    else
                    {
                        isOnSurface = false;
                    }
                }
            }

            accumulatedLength += Vector3.Distance(currentWorldPos, nextWorldPos);

            // 월드 -> 로컬로 변환해서 저장 (useWorldSpace=false 이므로)
            Vector3 nextLocalPos = trajectoryHelper.transform.InverseTransformPoint(nextWorldPos);
            localPoints.Add(nextLocalPos);

            lr.positionCount = localPoints.Count;
            lr.SetPositions(localPoints.ToArray());

            yield return new WaitForSeconds(growthInterval);
        } //while문 종료 지점


        if (localPoints.Count > 1)
        {
            Vector3 finalWindH = new Vector3(windVector.x, 0, windVector.z);
            Vector3 finalWindDir = (finalWindH.sqrMagnitude > 1e-6f) ? finalWindH.normalized : Vector3.zero;

            float windSpeed = finalWindH.magnitude;
            CreateMetaballSegments(trajectoryHelper, trajectoryHelper, localPoints, avgScale, Aw, waterValue, finalWindDir, windSpeed);
            BaseOfIcicles(trajectoryHelper, vIdx);

            OriginMCBlob metaballController = trajectoryHelper.GetComponent<OriginMCBlob>();
            if (metaballController == null)
            {
                metaballController = trajectoryHelper.AddComponent<OriginMCBlob>();

            }

            var colliders = trajectoryHelper.GetComponentsInChildren<SphereCollider>();
            if (colliders == null || colliders.Length == 0)
            {
                // 안전장치
                metaballController.gridSize = new Vector3(0.2f, 0.6f, 0.2f);
                metaballController.dimX = 32; metaballController.dimY = 64; metaballController.dimZ = 32;
            }
            else
            {
                // 1) 로컬 bounds 계산
                Bounds b = new Bounds(colliders[0].transform.localPosition, Vector3.zero);
                float maxR = 0f;

                for (int j = 0; j < colliders.Length; j++)
                {
                    var sc = colliders[j];
                    float r = sc.radius;
                    maxR = Mathf.Max(maxR, r);

                    b.Encapsulate(new Bounds(sc.transform.localPosition, Vector3.one * (2.5f * r)));
                }

                Vector3 centerLocal = b.center;

                // trajectoryHelper를 centerLocal 만큼 월드에서 이동
                Vector3 centerWorld = trajectoryHelper.transform.TransformPoint(centerLocal);
                trajectoryHelper.transform.position = centerWorld;

                for (int j = 0; j < colliders.Length; j++)
                {
                    colliders[j].transform.localPosition -= centerLocal;
                }

                for (int p = 0; p < localPoints.Count; p++)
                {
                    localPoints[p] -= centerLocal;
                }
                lr.SetPositions(localPoints.ToArray());

                float paddingXZ = Mathf.Max(maxR * 4.0f, 0.10f);
                float paddingY = Mathf.Max(maxR * 6.0f, 0.20f);

                Vector3 physicalSize = new Vector3(
                    b.size.x + paddingXZ,
                    b.size.y + paddingY,
                    b.size.z + paddingXZ
                );

                // 4) voxelSize로 dim 결정
                float voxelSize = 0.005f;

                int rx = Mathf.Clamp(Mathf.CeilToInt(physicalSize.x / voxelSize), 16, 64);
                int rz = Mathf.Clamp(Mathf.CeilToInt(physicalSize.z / voxelSize), 16, 64);
                int ry = Mathf.Clamp(Mathf.CeilToInt(physicalSize.y / voxelSize), 32, 128);

                // 5) voxel 총량 제한
                int maxVoxels = 800_000;
                long voxels = (long)rx * ry * rz;
                if (voxels > maxVoxels)
                {
                    float s = Mathf.Pow((float)maxVoxels / (float)voxels, 1f / 3f);
                    rx = Mathf.Max(32, Mathf.FloorToInt(rx * s));
                    ry = Mathf.Max(64, Mathf.FloorToInt(ry * s));
                    rz = Mathf.Max(32, Mathf.FloorToInt(rz * s));
                }

                // 6) 적용
                metaballController.gridSize = physicalSize;
                metaballController.dimX = rx;
                metaballController.dimY = ry;
                metaballController.dimZ = rz;
                metaballController.isoLevel = 0.3f;
            }

            try
            {
                var refreshMethod = metaballController.GetType().GetMethod("RefreshBlobList");
                if (refreshMethod != null) refreshMethod.Invoke(metaballController, null);

                metaballController.GenerateMesh();
                MeshRenderer mr = trajectoryHelper.GetComponent<MeshRenderer>();
                if (mr == null) mr = trajectoryHelper.AddComponent<MeshRenderer>();
                if (ice != null)
                {
                    mr.material = ice;
                }
                else
                {
                    mr.material = new Material(Shader.Find("Standard"));
                }
                Debug.Log($"고드름 {icicleIndex} 메쉬 생성 완료: Res({metaballController.dimX}x{metaballController.dimY}x{metaballController.dimZ})");
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"고드름 {icicleIndex} 메쉬 생성 중 오류: {e.Message}");
            }
        }
    }


    // Drip region 판정 함수 (클래스에 추가)
    private bool IsDripRegion(Vector3 worldPosition, Vector3 worldNormal)
    {
        // 중력 방향 (레이 방향)
        Vector3 worldG = raycastSource != null
            ? raycastSource.TransformDirection(Vector3.down).normalized
            : Vector3.down;

        // normal과 중력의 내적
        float dotProduct = Vector3.Dot(worldNormal, worldG);
        float angleThreshold = Mathf.Cos(dripLimitAngle * Mathf.Deg2Rad);

        // dotProduct >= threshold: 표면이 충분히 아래를 향함 = drip region
        return dotProduct >= angleThreshold;
    }

    private void CreateMetaballSegments(GameObject metaballIcicle, GameObject trajectoryHelper, List<Vector3> trajectory,
        float avgScale, float Aw, float waterValue, Vector3 windDir, float windSpeed)
    {
        if (trajectory == null || trajectory.Count <= 1) return;

        float windBias = IciclePhysics.CalculateIU(windSpeed);
        // 1) 전체 길이 & 구간 길이들
        float totalLength = 0f;
        var segLen = new List<float>(trajectory.Count - 1);
        for (int i = 0; i < trajectory.Count - 1; i++)
        {
            float d = Vector3.Distance(trajectory[i], trajectory[i + 1]);
            segLen.Add(d);
            totalLength += d;
        }
        if (totalLength <= 1e-6f) return;

        float acc = 0f;
        int segIdx = 0;
        float segAcc = 0f;

        float overlapBody = 0.55f;  // spacing = r * overlap
        float overlapTip = 0.30f;  // tip에서 더 촘촘
        float tipZone = 0.1f;  // 마지막 25% 구간
        float minSpacing = 0.004f;

        int safety = 0;
        int maxBalls = 8000;
        int bodyBallIndex = 0;
        // 시작점
        Place(trajectory[0], 0f, 0);

        while (acc < totalLength - 1e-6f && safety++ < maxBalls)
        {
            float t01 = acc / totalLength;
            float tipT = Mathf.InverseLerp(1f - tipZone, 1f, t01);                 // tip 구간만 0~1
            float overlap = Mathf.Lerp(overlapBody, overlapTip, Mathf.SmoothStep(0, 1, tipT));

            float icicleRadiusAtPosition = IcicleProfile(acc, totalLength, windSpeed);
            float spacing = Mathf.Max(minSpacing, icicleRadiusAtPosition * overlap);

            float target = Mathf.Min(acc + spacing, totalLength);

            while (segIdx < segLen.Count && segAcc + segLen[segIdx] < target)
            {
                segAcc += segLen[segIdx];
                segIdx++;
            }
            if (segIdx >= segLen.Count) break;

            float u = (target - segAcc) / Mathf.Max(1e-6f, segLen[segIdx]);
            Vector3 p = Vector3.Lerp(trajectory[segIdx], trajectory[segIdx + 1], u);
            int currentSegIdx = Mathf.Clamp(segIdx, 0, trajectory.Count - 2);
            Place(p, target, currentSegIdx);
            acc = target;
        }

        Vector3 tipLocal = trajectory[trajectory.Count - 1];
        int tipSegIdx = Mathf.Max(0, trajectory.Count - 2);
        Place(tipLocal, totalLength, tipSegIdx);

        if (trajectory.Count >= 2)
        {
            Vector3 prevLocal = trajectory[trajectory.Count - 2];
            Vector3 tipDirLocal = (tipLocal - prevLocal).normalized;

            float capR = Mathf.Max(minRadius, tipRadius);

            PlaceInternal(tipLocal, totalLength, capR, tipSegIdx);
            PlaceInternal(tipLocal - tipDirLocal * (capR * 0.35f), totalLength, capR * 0.9f, tipSegIdx);
        }

        void Place(Vector3 localPosInHelper, float curX, int currentSegIdx) => PlaceInternal(localPosInHelper, curX, -1f, currentSegIdx);

        Vector3 GetLocalPointAtDistance(float dist)
        {
            dist = Mathf.Clamp(dist, 0f, totalLength);

            float running = 0f;

            for (int i = 0; i < segLen.Count; i++)
            {
                float d = segLen[i];

                if (running + d >= dist)
                {
                    float u = (dist - running) / Mathf.Max(1e-6f, d);
                    return Vector3.Lerp(trajectory[i], trajectory[i + 1], u);
                }

                running += d;
            }

            return trajectory[trajectory.Count - 1];
        }

        void PlaceInternal(Vector3 localPosInHelper, float curX, float overrideRadius, int currentSegIdx)
        {
            Vector3 pointInWorld = trajectoryHelper.transform.TransformPoint(localPosInHelper);

            // 현재 구간의 원통 축(tangent)
            Vector3 tangent = Vector3.down;
            if (currentSegIdx < trajectory.Count - 1)
            {
                tangent = trajectoryHelper.transform.TransformDirection(
                    (trajectory[currentSegIdx + 1] - trajectory[currentSegIdx]).normalized);
            }

            // tangent에 수직인 단면 좌표계 생성
            Vector3 ortho1 = Vector3.Cross(tangent, Vector3.right);
            if (ortho1.sqrMagnitude < 1e-6f)
                ortho1 = Vector3.Cross(tangent, Vector3.up);
            ortho1.Normalize();

            Vector3 ortho2 = Vector3.Cross(tangent, ortho1).normalized;

            //바람을 단면 평면에 투영
            Vector3 windNorm = (windDir.sqrMagnitude > 1e-6f) ? windDir.normalized : Vector3.zero;
            Vector3 windCross = Vector3.ProjectOnPlane(windNorm, tangent);
            float crossWindAmount = windCross.magnitude;

            Vector3 windCrossNorm = (crossWindAmount > 1e-6f)
                ? windCross.normalized
                : Vector3.zero;


            float baseRadius = (overrideRadius > 0f)
                ? overrideRadius
                : IcicleProfile(curX, totalLength, windSpeed, crossWindAmount);

            float lengthFactor = Mathf.Clamp01(curX / Mathf.Max(0.0001f, totalLength));
            float rootThickness = Mathf.Lerp(waterValue, 0f, lengthFactor);
            baseRadius *= (1.0f + rootThickness);

            float rootAw = Aw;
            float windDrivenAw = 0.85f;
            float referenceWindSpeed = 10.0f;
            float windStrength = Mathf.Clamp01((windSpeed * windSpeed) / (referenceWindSpeed * referenceWindSpeed));

            float bodyBiasLerp = lengthFactor * windStrength;
            float currentBodyAw = Mathf.Lerp(rootAw, windDrivenAw, bodyBiasLerp);

            float volumeIncrease = 1.0f + (Mathf.Abs(currentBodyAw) * windBias);
            float finalRadius = baseRadius * volumeIncrease;

            //기존 편향은 유지하되, 단면 투영 바람 방향으로만 이동
            float offsetAmount = baseRadius * currentBodyAw * windBias * crossWindAmount;
            Vector3 asymmetryOffset = (crossWindAmount > 1e-6f) ? windCrossNorm * offsetAmount : Vector3.zero;
            pointInWorld += asymmetryOffset;

            bool shouldDebugDraw = debugWindNormal &&
                                   (debugEveryNthBall <= 1 || generatedIcicleSegments.Count % debugEveryNthBall == 0);

            if (shouldDebugDraw)
            {
                // tangent 축: 노랑
                Debug.DrawLine(
                    pointInWorld - tangent * debugAxisLength,
                    pointInWorld + tangent * debugAxisLength,
                    Color.yellow,
                    debugDuration,
                    false
                );

                // 단면에 투영된 wind 방향: 하늘
                if (crossWindAmount > 1e-6f)
                {
                    Debug.DrawLine(
                        pointInWorld,
                        pointInWorld + windCrossNorm * debugAxisLength * 1.3f,
                        Color.cyan,
                        debugDuration,
                        false
                    );
                }

                // ortho 축 2개: 흰색 / 회색
                Debug.DrawLine(
                    pointInWorld,
                    pointInWorld + ortho1 * debugAxisLength,
                    Color.white,
                    debugDuration,
                    false
                );

                Debug.DrawLine(
                    pointInWorld,
                    pointInWorld + ortho2 * debugAxisLength,
                    Color.gray,
                    debugDuration,
                    false
                );

                // 단면 샘플 normal들: 풍상=빨강, 풍하=파랑
                float visualAw = Mathf.Abs(currentBodyAw);

                for (int s = 0; s < debugSide; s++)
                {
                    float theta = s * (2f * Mathf.PI) / debugSide;
                    Vector3 surfaceNormal =
                        (ortho1 * Mathf.Cos(theta) + ortho2 * Mathf.Sin(theta)).normalized;

                    // +1 = 풍상 / -1 = 풍하
                    float signedDot = (crossWindAmount > 1e-6f)
                        ? Vector3.Dot(surfaceNormal, -windCrossNorm)
                        : 0f;

                    float directionalWeight = 1.0f + visualAw * (-signedDot);
                    directionalWeight = Mathf.Max(0.2f, directionalWeight);

                    float lineLen = debugNormalLength * directionalWeight;

                    Color c = (crossWindAmount > 1e-6f)
                        ? Color.Lerp(Color.blue, Color.red, (signedDot + 1f) * 0.5f)
                        : Color.green;

                    Debug.DrawLine(
                        pointInWorld,
                        pointInWorld + surfaceNormal * lineLen,
                        c,
                        debugDuration,
                        false
                    );
                }
            }


            if (positionNoiseAmount > 0f)
            {
                float t01 = (totalLength <= 1e-6f) ? 0f : (curX / totalLength);
                float noiseScale = Mathf.Lerp(1f, 0f, Mathf.SmoothStep(0.7f, 1f, t01));
                Vector2 rnd = Random.insideUnitCircle * (positionNoiseAmount * noiseScale);
                Vector3 off = trajectoryHelper.transform.right * rnd.x + trajectoryHelper.transform.up * rnd.y;
                pointInWorld += off;
            }

            GameObject g = new GameObject($"icicleSeg_{generatedIcicleSegments.Count}");
            g.transform.SetParent(metaballIcicle.transform, true);
            g.transform.position = pointInWorld;
            g.transform.rotation = Quaternion.identity;
            g.transform.localScale = Vector3.one;

            SphereCollider sc = g.AddComponent<SphereCollider>();
            sc.isTrigger = true;
            sc.radius = finalRadius;
            generatedIcicleSegments.Add(g);

            // 추가 Ripple metaball 배치
            bool canAddRipple =
              enableRippleMetaball &&
              overrideRadius < 0f &&
              windSpeed > 0.05f &&
              rippleSegmentCount > 0 &&
              rippleSegmentInterval > 0 &&
              bodyBallIndex % rippleSegmentInterval == 0 &&
              lengthFactor > 0.12f &&
              lengthFactor < 0.78f;

            if (canAddRipple)
            {
                // 현재 중심 메타볼과 다음 위치 사이에 리플 띠를 생성하기 위한 다음 위치 계산
                float bandStep = Mathf.Max(minSpacing, finalRadius * 0.4f);
                float nextX = Mathf.Min(curX + bandStep, totalLength);

                if (nextX > curX + 1e-5f)
                {
                    Vector3 nextLocal = GetLocalPointAtDistance(nextX);
                    Vector3 nextPointWorld = trajectoryHelper.transform.TransformPoint(nextLocal);

                    // 현재 중심 메타볼에 적용된 비대칭 오프셋과 비슷하게 맞춰줌
                    nextPointWorld += asymmetryOffset;

                    float nextLengthFactor = Mathf.Clamp01(nextX / Mathf.Max(0.0001f, totalLength));

                    float nextBaseRadius = IcicleProfile(nextX, totalLength, windSpeed, crossWindAmount);

                    float nextRootThickness = Mathf.Lerp(waterValue, 0f, nextLengthFactor);
                    nextBaseRadius *= (1.0f + nextRootThickness);

                    float nextBodyBiasLerp = nextLengthFactor * windStrength;
                    float nextCurrentBodyAw = Mathf.Lerp(rootAw, windDrivenAw, nextBodyBiasLerp);

                    float nextVolumeIncrease = 1.0f + (Mathf.Abs(nextCurrentBodyAw) * windBias);
                    float nextFinalRadius = nextBaseRadius * nextVolumeIncrease;

                    AddRippleMetaballsBetweenSegments(
                        metaballIcicle,
                        pointInWorld,
                        nextPointWorld,
                        finalRadius,
                        nextFinalRadius,
                        lengthFactor,
                        currentBodyAw,
                        windSpeed
                    );
                }
            }
            bodyBallIndex++; 
            void AddRippleMetaballsBetweenSegments(
            GameObject parentIcicle,
            Vector3 currentPointWorld,
            Vector3 nextPointWorld,
            float currentRadius,
            float nextRadius,
            float lengthFactor,
            float currentBodyAw,
            float windSpeed
        )
            {
                if (parentIcicle == null) return;
                if (rippleSegmentCount <= 0) return;

                Vector3 tangent = nextPointWorld - currentPointWorld;
                if (tangent.sqrMagnitude < 1e-6f) return;
                tangent.Normalize();

                // 현재 세그먼트와 다음 세그먼트 사이의 중간 단면
                Vector3 bridgeCenter = (currentPointWorld + nextPointWorld) * 0.5f;
                float mainRadius = (currentRadius + nextRadius) * 0.5f;

                // 중간 단면의 로컬 원통 좌표계
                Vector3 ortho1 = Vector3.Cross(tangent, Vector3.right);
                if (ortho1.sqrMagnitude < 1e-6f)
                    ortho1 = Vector3.Cross(tangent, Vector3.up);
                ortho1.Normalize();

                Vector3 ortho2 = Vector3.Cross(tangent, ortho1).normalized;

                // 바람을 중간 단면 평면에 투영
                Vector3 windH = new Vector3(windVector.x, 0f, windVector.z);
                Vector3 windNorm = (windH.sqrMagnitude > 1e-6f) ? windH.normalized : Vector3.zero;

                Vector3 windCross = Vector3.ProjectOnPlane(windNorm, tangent);
                float crossWindAmount = windCross.magnitude;

                if (crossWindAmount < 0.02f)
                {
                    Debug.Log($"[Ripple SKIP] crossWind too small: {crossWindAmount:F3}");
                    return;
                }

                Vector3 windCrossNorm = windCross.normalized;
                float effectiveCrossWind = Mathf.Clamp(crossWindAmount, 0.25f, 1.0f);

                // 풍속 영향
                float windStrength = Mathf.Clamp01(
                    (windSpeed * windSpeed) /
                    Mathf.Max(0.0001f, rippleReferenceWindSpeed * rippleReferenceWindSpeed)
                );

                float awEffect = Mathf.Clamp01(Mathf.Abs(currentBodyAw));
                float asymEffect = Mathf.Clamp01(
                     windStrength * effectiveCrossWind * Mathf.Lerp(0.7f, 1.2f, awEffect)
                 );

                // 끝부분으로 갈수록 리플 감소
                float rootFade = Mathf.SmoothStep(rippleRootFadeStart, rippleRootFadeEnd, lengthFactor);
                float tipFade = 1f - Mathf.SmoothStep(rippleTipFadeStart, 1f, lengthFactor);

                float rawBodyFade = rootFade * tipFade;

                // 끝부분은 제외
                if (lengthFactor > 0.90f)
                {
                    Debug.Log($"[Ripple SKIP] too close to tip, length={lengthFactor:F2}");
                    return;
                }

                // 리플이 너무 작아져서 사라지는 것 방지
                float bodyFade = Mathf.Max(rawBodyFade, 0.25f);


                float neutralRatio = rippleNeutralRatio;

                float dynamicWindwardRatio = Mathf.Lerp(
                    neutralRatio,
                    windwardRippleRatio,
                    asymEffect
                );

                float dynamicLeewardRatio = Mathf.Lerp(
                    neutralRatio,
                    leewardRippleRatio,
                    asymEffect
                );

                // 풍상은 촘촘하게, 풍하는 듬성하고 크게
                int windwardCount = Mathf.Max(
                     6,
                     Mathf.RoundToInt(Mathf.Lerp(rippleSegmentCount + 2, rippleSegmentCount + 4, asymEffect))
                );

                // 풍하: 크지만 듬성하게
                int leewardCount = Mathf.Max(
                    3,
                    Mathf.RoundToInt(Mathf.Lerp(rippleSegmentCount * 0.6f, rippleSegmentCount * 0.45f, asymEffect))
                );

                float windwardArcDeg = Mathf.Lerp(150f, 120f, asymEffect);
                float leewardArcDeg = Mathf.Lerp(180f, 160f, asymEffect);

                // windCrossNorm 방향 = 풍하
                Vector3 windwardCenterDir = -windCrossNorm;
                Vector3 leewardCenterDir = windCrossNorm;

                SpawnRippleArc(
                    windwardCenterDir,
                    windwardCount,
                    windwardArcDeg,
                    dynamicWindwardRatio,
                    0.35f,
                    1.0f,
                    "WindwardRipple"
                );

                SpawnRippleArc(
                    leewardCenterDir,
                    leewardCount,
                    leewardArcDeg,
                    dynamicLeewardRatio,
                    0.55f,
                    1.15f,
                    "LeewardRipple"
                );

                void SpawnRippleArc(
                    Vector3 centerDir,
                    int count,
                    float arcDegrees,
                    float radiusRatio,
                    float protrudeScale,
                    float radiusBoost,
                    string label
                )
                {
                    centerDir.Normalize();

                    float centerAngle = Mathf.Atan2(
                        Vector3.Dot(centerDir, ortho2),
                        Vector3.Dot(centerDir, ortho1)
                    );

                    float arcRad = arcDegrees * Mathf.Deg2Rad;

                    for (int i = 0; i < count; i++)
                    {
                        float t = (count <= 1) ? 0.5f : (float)i / (count - 1);
                        float angle = centerAngle - arcRad * 0.5f + arcRad * t;

                        Vector3 dir =
                            Mathf.Cos(angle) * ortho1 +
                            Mathf.Sin(angle) * ortho2;

                        dir.Normalize();
                        //추가
                        float leewardT = Mathf.Clamp01(
                            (Vector3.Dot(dir, windCrossNorm) + 1f) * 0.5f
                        );

                        // 풍상 방향은 외곽을 줄이고, 풍하 방향은 외곽을 확장
                        float envelopeScale = Mathf.Lerp(
                            1f - windwardBodyShrink * asymEffect,
                            1f + leewardBodyExpand * asymEffect,
                            leewardT
                        );

                        bool isLeeward = label.Contains("Leeward");

                        // bodyFade를 그대로 곱하지 않고 완화
                        float visibleFade = Mathf.Lerp(0.65f, 1.0f, bodyFade);

                        float rippleRadius = mainRadius * radiusRatio * visibleFade;

                        float sideScale = isLeeward
                            ? Mathf.Lerp(1.0f, 1.25f, asymEffect)
                            : Mathf.Lerp(1.0f, 1.05f, asymEffect);

                        rippleRadius *= sideScale;

                        // 너무 작지도, 너무 크지도 않게 mainRadius 기준으로 제한
                        float minRatio = isLeeward ? 0.09f : 0.07f;
                        float maxRatio = isLeeward ? 0.42f : 0.28f;

                        rippleRadius = Mathf.Clamp(rippleRadius, mainRadius * minRatio, mainRadius * maxRatio);

                        if (rippleRadius < minRadius * 0.5f)
                            continue;

                        float centerShift = mainRadius * leewardCenterShift * asymEffect * bodyFade * 0.5f;
                        Vector3 biasedCenter = bridgeCenter + windCrossNorm * centerShift;

                        // 몸통 표면에 붙게 배치
                        float protrude = rippleRadius * (1f - rippleEmbedRatio);
                        float centerDistance = mainRadius + protrude;

                        Vector3 rippleWorldPos = biasedCenter + dir * centerDistance;

                        GameObject ripple = new GameObject($"{label}_{generatedIcicleSegments.Count}");
                        ripple.transform.SetParent(parentIcicle.transform, true);
                        ripple.transform.position = rippleWorldPos;
                        ripple.transform.rotation = Quaternion.identity;
                        ripple.transform.localScale = Vector3.one;

                        SphereCollider rippleCollider = ripple.AddComponent<SphereCollider>();
                        rippleCollider.isTrigger = true;
                        rippleCollider.radius = rippleRadius;

                        generatedIcicleSegments.Add(ripple);

                        if (debugWindNormal)
                        {
                            Color debugColor = label.Contains("Leeward")
                                ? Color.blue
                                : Color.red;

                            Debug.DrawLine(
                                biasedCenter, 
                                rippleWorldPos,
                                debugColor,
                                debugDuration,
                                false
                            );
                        }
                    }
                }
            }

        }
    }


    //프로파일 함수 구현 4.4.1
    private float IcicleProfile(float x, float L, float windSpeed, float crossWindAmount = 0f)
    {
        if (L <= 1e-6f) return tipRadius;

        x = Mathf.Clamp(x, 0f, L);

        float windFactor = Mathf.Max(1f, windSpeed);
        float k = 0.8f;

        // 기존 dot 대신 "단면에 바람이 얼마나 걸리는가"만 반영
        float dynamicFrequency = rippleFrequency * windFactor * (1f + crossWindAmount * k);

        float R =
            tipRadius +
            (L - x) * taper +
            rippleAmplitude * Mathf.Sin(x * dynamicFrequency);

        return Mathf.Clamp(R, minRadius, 2.0f);
    }

    // 4.4.3 Base of Icicle 
    private void BaseOfIcicles(GameObject parentIcicle, int mainDripPointIndex)
    {

        float eb = baseSpreadDistance;    // influence radius (e_b)
        int nmb = baseMetaballCount;      // number of base metaballs (n_mb)
        if (parentIcicle == null || eb <= 0f || nmb <= 0) return;

        //물 계수 정규화 
        float denom = 1f;
        if (waterCoeff != null && waterCoeff.Length > 0)
        {
            float[] tmp = (float[])waterCoeff.Clone();
            System.Array.Sort(tmp);
            int p = Mathf.Clamp(Mathf.FloorToInt(tmp.Length * 0.95f), 0, tmp.Length - 1);
            denom = Mathf.Max(1e-6f, tmp[p]);
        }


        Vector3 pLocal = vertices[mainDripPointIndex];
        Vector3 nLocal = normals[mainDripPointIndex].normalized;
        Vector3 centerWorld = transform.TransformPoint(pLocal + nLocal * 0.003f);

        // 범위 eb 안에서 포인트 뿌리기 
        var nearby = new List<int>();
        var geod = new Dictionary<int, float>(256);
        var q = new Queue<int>(256);
        int start = mainDripPointIndex;

        geod[start] = 0f;
        q.Enqueue(start);

        while (q.Count > 0)
        {
            int v = q.Dequeue();
            float dv = geod[v];
            if (dv > eb) continue;

            if (v != start)
                nearby.Add(v);

            if (!vertexNeighbors.TryGetValue(v, out var nbs)) continue;
            foreach (int u in nbs)
            {
                float w = Vector3.Distance(vertices[v], vertices[u]);
                float nd = dv + w;
                if (nd <= eb && (!geod.ContainsKey(u) || nd < geod[u]))
                {
                    geod[u] = nd;
                    q.Enqueue(u);
                }
            }
        }

        if (nearby.Count == 0) nearby.Add(start);


        nearby.Sort((a, b) => geod[a].CompareTo(geod[b]));
        int take = Mathf.Min(nmb, nearby.Count);

        float wcDrip = Mathf.Clamp01(waterCoeff[start] / denom);

        //드립포인트 주변으로 메타볼 생성
        for (int k = 0; k < take; k++)
        {
            int idx = nearby[k];
            Vector3 vLocal = vertices[idx];
            Vector3 vNorm = normals[idx].normalized;


            float dLocal = geod.TryGetValue(idx, out var dd) ? dd : eb;
            float numer = Mathf.Max(0f, eb - dLocal);
            float falloff = (numer * numer) / (eb * eb);
            float rb = falloff * wcDrip * 0.2f;

            Vector3 worldPos = transform.TransformPoint(vLocal);


            // Create metaball object
            GameObject seg = new GameObject($"IcicleBase_{idx}");
            seg.transform.SetParent(parentIcicle.transform, true);
            seg.transform.position = worldPos;
            seg.transform.rotation = Quaternion.identity;
            seg.transform.localScale = Vector3.one;

            SphereCollider sc = seg.AddComponent<SphereCollider>();
            sc.isTrigger = true;
            sc.radius = rb;

            generatedIcicleSegments.Add(seg);
        }
    }

    private void GlazeIce()
    {
        // 1) 초기화/정리
        if (glazeIceContainer != null) DestroyImmediate(glazeIceContainer);
        Transform existing = meshFilterComponent.transform.Find("GlazeIce");
        if (existing != null) DestroyImmediate(existing.gameObject);

        if (vertices == null || normals == null || waterCoeff == null) return;
        if (triangleAreas == null || triangleAreas.Length == 0) CalculateMeshAreas();

        // 2) 중력 방향 (로컬)
        Vector3 worldGravityDir = raycastSource.TransformDirection(Vector3.down).normalized;
        Vector3 localGravityDir = transform.InverseTransformDirection(worldGravityDir).normalized;

        // 3) 높이 범위 계산 (물 있는 곳만)
        float minH = float.PositiveInfinity;
        float maxH = float.NegativeInfinity;
        for (int i = 0; i < vertices.Length; i++)
        {
            if (waterCoeff[i] <= 0f) continue;
            float h = Vector3.Dot(vertices[i], -localGravityDir);
            if (h < minH) minH = h;
            if (h > maxH) maxH = h;
        }
        if (!float.IsFinite(minH)) return;
        float heightRange = Mathf.Max(1e-6f, maxH - minH);

        // 4) 컨테이너 생성
        glazeIceContainer = new GameObject("GlazeIce");
        glazeIceContainer.transform.SetParent(meshFilterComponent.transform, false);
        glazeIceContainer.transform.localPosition = Vector3.zero;
        glazeIceContainer.transform.localRotation = Quaternion.identity;
        glazeIceContainer.transform.localScale = Vector3.one;

        OriginMCBlob glazeMetaball = glazeIceContainer.AddComponent<OriginMCBlob>();
        glazeIceContainer.AddComponent<MeshFilter>();
        glazeIceContainer.AddComponent<MeshRenderer>();

        // 블루 노이즈 샘플링
        int targetCount = Mathf.Max(1, Mathf.RoundToInt(ngi));
        List<IceCandidate> points = GenerateBlueNoiseSamples(targetCount, localGravityDir, minH, heightRange);
        if (points == null || points.Count == 0) return;

        float surfaceOffsetFactor = 0.02f;


        for (int i = 0; i < points.Count; i++)
        {
            var p = points[i];

            float rLocal = p.radius; //메쉬의 로컬 기준 반지름
            Vector3 pos = p.localPos + p.normal * (rLocal * surfaceOffsetFactor);

            GameObject seg = new GameObject($"GlazeIce_Seg_{i}");
            seg.transform.SetParent(glazeIceContainer.transform, false);
            seg.transform.localPosition = pos;
            seg.transform.localRotation = Quaternion.identity;
            seg.transform.localScale = Vector3.one;

            SphereCollider sc = seg.AddComponent<SphereCollider>();
            sc.isTrigger = true;
            sc.radius = rLocal * 1.2f;
        }

        //그리드 설정
        var colliders = glazeIceContainer.GetComponentsInChildren<SphereCollider>();
        if (colliders == null || colliders.Length == 0) return;

        Bounds b = new Bounds(colliders[0].transform.localPosition, Vector3.zero);
        float maxR = 0f;

        foreach (var sc in colliders)
        {
            float r = sc.radius;
            maxR = Mathf.Max(maxR, r);

            Vector3 p = sc.transform.localPosition;
            b.Encapsulate(new Bounds(p, Vector3.one * (2f * r)));
        }

        // 컨테이너를 그리드 중심으로 이동
        Vector3 center = b.center;
        glazeIceContainer.transform.localPosition = center;

        foreach (var sc in colliders)
        {
            sc.transform.localPosition -= center;
        }

        float padding = Mathf.Max(maxR * 1.0f, 0.02f);
        Vector3 size = b.size + Vector3.one * padding;

        float targetVoxelSize = Mathf.Max(maxR * 0.15f, 0.001f);

        // 2. 결정된 Voxel 크기로 각 축의 해상도(dim) 계산
        int rx = Mathf.CeilToInt(size.x / targetVoxelSize);
        int ry = Mathf.CeilToInt(size.y / targetVoxelSize);
        int rz = Mathf.CeilToInt(size.z / targetVoxelSize);

        int maxAllowedDim = 128;
        int currentMax = Mathf.Max(rx, Mathf.Max(ry, rz));

        if (currentMax > maxAllowedDim)
        {
            float scaleRatio = (float)maxAllowedDim / currentMax;
            rx = Mathf.FloorToInt(rx * scaleRatio);
            ry = Mathf.FloorToInt(ry * scaleRatio);
            rz = Mathf.FloorToInt(rz * scaleRatio);
        }

        // 4. 최종 결과 적용
        glazeMetaball.gridSize = size;
        glazeMetaball.dimX = Mathf.Max(rx, 64);
        glazeMetaball.dimY = Mathf.Max(ry, 64);
        glazeMetaball.dimZ = Mathf.Max(rz, 64);

        // 5. 임계값 조절 
        glazeMetaball.isoLevel = 0.3f;

        glazeMetaball.RefreshBlobList();
        glazeMetaball.GenerateMesh();
    }


    //블루노이즈 샘플링 
    private List<IceCandidate> GenerateBlueNoiseSamples(int targetCount, Vector3 localGravityDir, float minH, float heightRange)
    {

        int sampleFactor = 8;
        int inputCount = targetCount * sampleFactor;
        int maxAttempts = inputCount * 5;

        List<IceCandidate> candidates = new List<IceCandidate>(inputCount);
        int[] triangles = meshFilterComponent.sharedMesh.triangles;

        int generated = 0;
        int attempts = 0;

        // (A) 무작위 샘플링
        while (generated < inputCount && attempts < maxAttempts)
        {
            attempts++;

            float randomVal = Random.Range(0, totalArea);
            int triIndex = System.Array.BinarySearch(cumulativeAreas, randomVal);
            if (triIndex < 0) triIndex = ~triIndex;
            if (triIndex >= triangleAreas.Length) triIndex = triangleAreas.Length - 1;

            int i0 = triangles[triIndex * 3];
            int i1 = triangles[triIndex * 3 + 1];
            int i2 = triangles[triIndex * 3 + 2];

            float r1 = Mathf.Sqrt(Random.Range(0f, 1f));
            float r2 = Random.Range(0f, 1f);
            float m1 = 1 - r1;
            float m2 = r1 * (1 - r2);
            float m3 = r1 * r2;

            Vector3 vLocal = vertices[i0] * m1 + vertices[i1] * m2 + vertices[i2] * m3;
            float interpolatedWater = waterCoeff[i0] * m1 + waterCoeff[i1] * m2 + waterCoeff[i2] * m3;

            Vector3 nLocal = normals[i0] * m1 + normals[i1] * m2 + normals[i2] * m3;
            nLocal.Normalize();

            float distAlongGravity = Vector3.Dot(vLocal, -localGravityDir);
            float dNorm = Mathf.Clamp01((distAlongGravity - minH) / heightRange);

            float dDown = dNorm;
            float dUp = 1f - dNorm;
            float lt = Mathf.Clamp01(lifeTime);

            float rGI = minGI + scaling * (dUp * lt + dDown * (1f - lt));


            if (rGI <= 0f) rGI = minGI + 0.003f;

            candidates.Add(new IceCandidate
            {
                index = generated,
                localPos = vLocal,
                normal = nLocal,
                waterAmount = Mathf.Max(interpolatedWater, 0.001f),  // 최솟값 보장
                radius = rGI,
                weight = 0f,
                eliminated = false
            });
            generated++;
        }

        if (candidates.Count <= targetCount) return candidates;

        float idealDistance = Mathf.Sqrt(totalArea / (targetCount * 0.866f));
        float limitDist = 0.9f * idealDistance;
        float limitDistSq = limitDist * limitDist;
        float alpha = 5f;

        // (C) Grid 생성
        float cellSize = limitDist;
        Dictionary<Vector3Int, List<int>> grid = new Dictionary<Vector3Int, List<int>>();

        Vector3Int GetGridPos(Vector3 pos)
        {
            return new Vector3Int(
                Mathf.FloorToInt(pos.x / cellSize),
                Mathf.FloorToInt(pos.y / cellSize),
                Mathf.FloorToInt(pos.z / cellSize)
            );
        }

        for (int i = 0; i < candidates.Count; i++)
        {
            Vector3Int gp = GetGridPos(candidates[i].localPos);
            if (!grid.ContainsKey(gp)) grid[gp] = new List<int>();
            grid[gp].Add(i);
        }

        // 초기 가중치 계산
        float[] currentWeights = new float[candidates.Count];

        for (int i = 0; i < candidates.Count; i++)
        {
            Vector3 pos = candidates[i].localPos;
            Vector3 norm = candidates[i].normal;
            Vector3Int gp = GetGridPos(pos);
            float w = 0f;

            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    for (int z = -1; z <= 1; z++)
                    {
                        Vector3Int neighborKey = gp + new Vector3Int(x, y, z);
                        if (!grid.TryGetValue(neighborKey, out var neighborList)) continue;

                        foreach (int j in neighborList)
                        {
                            if (i == j) continue;


                            if (Vector3.Dot(norm, candidates[j].normal) < 0.5f) continue;

                            float dSq = (pos - candidates[j].localPos).sqrMagnitude;
                            if (dSq < limitDistSq)
                            {
                                float d = Mathf.Sqrt(dSq);
                                float val = Mathf.Pow(1f - (d / limitDist), alpha);
                                w += val;
                            }
                        }
                    }
                }
            }
            var c = candidates[i];
            c.weight = w;
            candidates[i] = c;
            currentWeights[i] = w;
        }

        // 제거 루프
        int currentCount = candidates.Count;
        bool[] isEliminated = new bool[candidates.Count];

        while (currentCount > targetCount)
        {
            float maxW = -1f;
            int removeIdx = -1;

            for (int i = 0; i < candidates.Count; i++)
            {
                if (!isEliminated[i])
                {
                    if (currentWeights[i] > maxW)
                    {
                        maxW = currentWeights[i];
                        removeIdx = i;
                    }
                }
            }

            if (removeIdx == -1) break;

            isEliminated[removeIdx] = true;
            currentCount--;

            Vector3 removePos = candidates[removeIdx].localPos;
            Vector3 removeNorm = candidates[removeIdx].normal;
            Vector3Int gp = GetGridPos(removePos);

            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    for (int z = -1; z <= 1; z++)
                    {
                        Vector3Int neighborKey = gp + new Vector3Int(x, y, z);
                        if (!grid.TryGetValue(neighborKey, out var neighborList)) continue;

                        foreach (int j in neighborList)
                        {
                            if (isEliminated[j] || removeIdx == j) continue;

                            if (Vector3.Dot(removeNorm, candidates[j].normal) < 0.1f) continue;

                            float dSq = (removePos - candidates[j].localPos).sqrMagnitude;
                            if (dSq < limitDistSq)
                            {
                                float d = Mathf.Sqrt(dSq);
                                float val = Mathf.Pow(1f - (d / limitDist), alpha);
                                currentWeights[j] -= val;
                            }
                        }
                    }
                }
            }
        }

        List<IceCandidate> result = new List<IceCandidate>(targetCount);
        for (int i = 0; i < candidates.Count; i++)
        {
            if (!isEliminated[i])
            {
                result.Add(candidates[i]);
            }
        }

        if (result.Count < targetCount)
        {
            Debug.Log($"Gap Filling: {result.Count}/{targetCount}");

            int needed = targetCount - result.Count;
            int fillAttempts = 0;

            // 최소 거리를 더 완화
            float relaxedDistSq = limitDistSq * 0.2f;  // 절반의 절반의 절반

            while (result.Count < targetCount && fillAttempts < needed * 50)
            {
                fillAttempts++;

                float randomVal = Random.Range(0, totalArea);
                int triIndex = System.Array.BinarySearch(cumulativeAreas, randomVal);
                if (triIndex < 0) triIndex = ~triIndex;
                if (triIndex >= triangleAreas.Length) triIndex = triangleAreas.Length - 1;

                int i0 = triangles[triIndex * 3];
                int i1 = triangles[triIndex * 3 + 1];
                int i2 = triangles[triIndex * 3 + 2];

                float r1 = Mathf.Sqrt(Random.Range(0f, 1f));
                float r2 = Random.Range(0f, 1f);
                float m1 = 1 - r1;
                float m2 = r1 * (1 - r2);
                float m3 = r1 * r2;

                Vector3 vLocal = vertices[i0] * m1 + vertices[i1] * m2 + vertices[i2] * m3;
                Vector3 nLocal = normals[i0] * m1 + normals[i1] * m2 + normals[i2] * m3;
                nLocal.Normalize();

                float distAlongGravity = Vector3.Dot(vLocal, -localGravityDir);
                float dNorm = Mathf.Clamp01((distAlongGravity - minH) / heightRange);
                float dDown = dNorm;
                float dUp = 1f - dNorm;
                float lt = Mathf.Clamp01(lifeTime);
                float rGI = minGI + scaling * (dUp * lt + dDown * (1f - lt));

                if (rGI <= 0f) rGI = minGI + 0.003f;

                // 완화된 거리 체크
                bool tooClose = false;
                foreach (var existing in result)
                {
                    if ((vLocal - existing.localPos).sqrMagnitude < relaxedDistSq)
                    {
                        tooClose = true;
                        break;
                    }
                }        

                if (!tooClose)
                {
                    result.Add(new IceCandidate
                    {
                        index = result.Count,
                        localPos = vLocal,
                        normal = nLocal,
                        waterAmount = 0.001f,
                        radius = rGI,
                        weight = 0f,
                        eliminated = false
                    });
                }
            }
        }

        Debug.Log($"Glaze Ice 최종: {result.Count}/{targetCount}개 생성");

        return result;
    }

}
