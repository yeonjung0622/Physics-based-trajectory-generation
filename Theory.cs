
using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using UnityEngine;

public class Theory : MonoBehaviour
{
    //인스펙터 접근 불가 변수 (private)
    private MeshFilter meshFilterComponent;
    private Mesh mesh;
    private Vector3[] vertices;
    private Color[] colors;
    private float[] waterAmounts;
    //  private bool isSimulationRunning = false;

    //레이 회전 반영 관련 변수
    private Coroutine calculationCoroutine;
    private Quaternion lastRotation;
    public Transform raycastSource;
    //2단계에서 필요한 변수들
    private Vector3[] normals;
    public List<Vector3> dripPoints = new List<Vector3>();

    //3단계에서 필요한  변수
    public float maxIcicleLength = 40.0f;
    public Material blob;
    private List<LineRenderer> icicleRenderers = new List<LineRenderer>();

    // L-System 기반 궤적 계산용 내부 상태 변수
    private Stack<(Vector3 pos, Vector3 dir)> transformStack;
    private Vector3 currentSurfaceNormal;
    private Vector3 currentDir;
    private bool isFollowingSurface;

    //4단계 필요 변수
    public OriginMCBlob MetaballController;


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
    public int[] vertexIndex;

    //3단계에서 필요한 변수들 (전역변수)
    [Header("Icicle L-System Parameters")]
    public int iterations = 500;
    [Range(0.0f, 180.0f)]
    public float curvatureAngle = 25.0f;
    [Range(0.0f, 1.0f)]
    public float subdivisionProbability = 0.5f;
    [Range(0.0f, 360.0f)]
    public float userDispersionAngle = 137.5f;
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
    public float dropletRadius = 2.44f;
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
        if (raycastSource == null || raycastSource.rotation == lastRotation)
            if (raycastSource == null) return;

        // 1. 레이 컨트롤러(비 방향)가 바뀌었을 때만 물 계산 (기존 로직)
        float angleDiff = Quaternion.Angle(raycastSource.rotation, lastRotation);
        if (angleDiff > 1.0f)
        {
            if (calculationCoroutine != null) StopCoroutine(calculationCoroutine);
            calculationCoroutine = StartCoroutine(WaterCoefficientCalculateCoroutine());
            lastRotation = raycastSource.rotation;
        }

    }


    //처음 물 공급원 저장 함수
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
    }



    //2단계 고드름 드립 포인트 찾기
    public void DripPointsIdentification()
    {
        dripPoints.Clear();
        gizmoVertexIndices.Clear();

        Vector3 worldG = raycastSource ? raycastSource.TransformDirection(Vector3.down) : Vector3.down;
        Vector3 g = transform.InverseTransformDirection(worldG).normalized;

        List<int> dripRegionVertices = new List<int>();
        for (int i = 0; i < vertices.Length; i++)
        {
            if (waterCoeff[i] <= 0.001f) continue;
            float downDot = Vector3.Dot(normals[i], g);
            if (downDot < 0.2f || downDot > 0.75f) continue;
            dripRegionVertices.Add(i);
        }

        if (dripRegionVertices.Count == 0) return;

        // 랜덤하게 드립 포인트 선정
        for (int i = 0; i < dripRegionVertices.Count; i++)
        {
            int swap = Random.Range(i, dripRegionVertices.Count);
            (dripRegionVertices[i], dripRegionVertices[swap]) = (dripRegionVertices[swap], dripRegionVertices[i]);
        }

        int iciclesToGenerate = Mathf.Min(numberOfIcicles, dripRegionVertices.Count);
        for (int i = 0; i < iciclesToGenerate; i++)
        {
            int vIdx = dripRegionVertices[i];
            dripPoints.Add(vertices[vIdx]);
            gizmoVertexIndices.Add(vIdx);
        }

        // [수정] 여기서 바로 생성하지 않고, Update에서 바람 변화를 체크하거나 
        // 최초 1회만 생성하도록 함
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

    // 고드름 생성을 관리하는 코루틴
    private IEnumerator GenerateIcicles()
    {
        // 초기화 로직 (기존과 동일)
        foreach (LineRenderer lr in icicleRenderers) if (lr != null) Destroy(lr.gameObject);
        icicleRenderers.Clear();

        float avgScale = (transform.lossyScale.x + transform.lossyScale.y + transform.lossyScale.z) / 3.0f;

        // 모든 드립 포인트에서 동시에 자라게 하기 위해 리스트로 관리
        List<Coroutine> activeIcicles = new List<Coroutine>();

        for (int i = 0; i < numberOfIcicles && i < dripPoints.Count; i++)
        {
            // 각 고드름을 개별 코루틴으로 실행 (동시 성장)
            activeIcicles.Add(StartCoroutine(GrowSingleIcicleRealtime(i, avgScale)));
        }

        yield return null;
    }

    // 개별 고드름이 실시간으로 자라나는 코루틴
    private IEnumerator GrowSingleIcicleRealtime(int icicleIndex, float avgScale)
    {
        int vIdx = gizmoVertexIndices[icicleIndex];
        Vector3 worldDripPoint = transform.TransformPoint(vertices[vIdx]);
        Vector3 worldNormal = transform.TransformDirection(normals[vIdx]).normalized;
        Vector3 currentWorldPos = worldDripPoint + worldNormal * (0.01f * avgScale);

        // 1. LineRenderer 세팅
        GameObject trajectoryHelper = new GameObject($"IcicleLine_{icicleIndex}");
        trajectoryHelper.transform.position = currentWorldPos;
        trajectoryHelper.transform.parent = this.transform;
        LineRenderer lr = trajectoryHelper.AddComponent<LineRenderer>();
        icicleRenderers.Add(lr);

        lr.useWorldSpace = true; // 실시간 이동을 위해 월드 좌표계 사용
        lr.startWidth = 0.02f * avgScale;
        lr.endWidth = 0.005f * avgScale;
        lr.material = lineMaterial;
        lr.positionCount = 1;
        lr.SetPosition(0, currentWorldPos);

        List<Vector3> points = new List<Vector3> { currentWorldPos };

        // 2. 성장 상태 변수
        float current_tan_phi = 0.0f;
        float accumulatedLength = 0f;
        float waterValue = Mathf.Clamp01(waterCoeff[vIdx] / maxWaterAmount);
        float maxLength = maxIcicleLength * waterValue;

        bool isOnSurface = false;
        Vector3 currentNormal = Vector3.up;

        // 3. 실시간 성장 루프
        while (accumulatedLength < maxLength)
        {
            // 매 스텝마다 현재 바람 방향을 읽음 
            float currentWindSpeed = windVector.magnitude;
            Vector3 horizontalWindDir = new Vector3(windVector.x, 0, windVector.z).normalized;

            // 물리량 재계산 (바람이 바뀌었을 수 있으므로)
            float rMeters = dropletRadius / 1000f;
            float eta = IciclePhysics.CalculateEta(Temperature);
            float Re = IciclePhysics.CalculateReynolds(currentWindSpeed, rMeters, eta);
            float cd = IciclePhysics.CalculateCd(Re);
            float tan_theory = IciclePhysics.CalculateTanTheory(currentWindSpeed, rMeters, cd);
            float beta = IciclePhysics.ComputeBeta(v_speed_mm, segmentLength);

            // 물리 누적 업데이트
            float simDt = growthInterval; // 한 스텝의 시간
            float currentStepVal = current_tan_phi;
            float tracking = beta * (tan_theory - current_tan_phi) * simDt;
            current_tan_phi += tracking; //현재 스텝에서 다음 스텝 더하기 
            float nextStepVal = current_tan_phi;
            if (icicleIndex == 0)
            {
                Debug.Log($"[Step Logic] 현재값: {currentStepVal:F4} + 변화량: {tracking:F4} = 다음값: {nextStepVal:F4} (목표 이론각: {tan_theory:F4})");
            }

            // 다음 위치 계산
            Vector3 stepDir = (Vector3.down + horizontalWindDir * current_tan_phi).normalized;
            float stepLen = segmentLength;
            Vector3 nextWorldPos = currentWorldPos + stepDir * stepLen;

            // 표면 충돌 및 슬라이딩 
            RaycastHit hit;
            if (!isOnSurface)
            {
                if (Physics.Raycast(currentWorldPos, stepDir, out hit, stepLen * 1.2f, collisionLayers))
                {
                    nextWorldPos = hit.point + hit.normal * 0.02f;
                    currentNormal = hit.normal;
                    isOnSurface = true;
                }
            }
            else
            {
                if (IsDripRegion(currentWorldPos, currentNormal)) isOnSurface = false;
                else
                {
                    Vector3 slideDir = Vector3.ProjectOnPlane(stepDir, currentNormal).normalized;
                    if (Physics.Raycast(currentWorldPos + currentNormal * 0.02f, slideDir, out hit, stepLen * 1.2f, collisionLayers))
                    {
                        nextWorldPos = hit.point + hit.normal * 0.02f;
                        currentNormal = hit.normal;
                    }
                    else isOnSurface = false;
                }
            }

            // 데이터 업데이트
            accumulatedLength += Vector3.Distance(currentWorldPos, nextWorldPos);
            currentWorldPos = nextWorldPos;
            points.Add(currentWorldPos);

            // LineRenderer 업데이트
            lr.positionCount = points.Count;
            lr.SetPositions(points.ToArray());

            // 시간 지연 : 간격 설정으로 바람 방향 반영 후 궤적 다시 계산 가능
            yield return new WaitForSeconds(growthInterval);
        }
    }

    // Drip region 판정 함수 (클래스에 추가)
    private bool IsDripRegion(Vector3 worldPosition, Vector3 worldNormal)
    {
        // 중력 방향 (레이 방향)
        Vector3 worldGravity = raycastSource != null
            ? raycastSource.TransformDirection(Vector3.down).normalized
            : Vector3.down;

        // normal과 중력의 내적
        float dotProduct = Vector3.Dot(worldNormal, worldGravity);
        float angleThreshold = Mathf.Cos(dripLimitAngle * Mathf.Deg2Rad);

        // dotProduct >= threshold: 표면이 충분히 아래를 향함 = drip region
        return dotProduct >= angleThreshold;
    }

}
