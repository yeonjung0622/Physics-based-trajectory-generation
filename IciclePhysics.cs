using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class IciclePhysics
{
    private const float RHO_A = 1.2f;
    private const float RHO_W = 1000f;
    private const float G = 9.8f;
    private const float U_MAX = 15.0f; // 풍속 포화 기준값

    private static float lastU = -999f;
    private static float lastBeta = -1f;
    private static float lastV = -1f;

    // 1. 공기 점성도 계산 (Sutherland's Law)
    public static float CalculateEta(float tempCelsius)
    {
        float T = tempCelsius + 273.15f;
        return 1.716e-5f * Mathf.Pow(T / 273.15f, 1.5f) * (273.15f + 110.4f) / (T + 110.4f);
    }

    // 2. 레이놀즈 수 계산
    public static float CalculateReynolds(float U, float r, float eta)
    {
        if (eta <= 0f) return 0f;
        return (2f * RHO_A * r * U) / eta;
    }

    // 3. 항력 계수 계산
    public static float CalculateCd(float Re)
    {
        if (Re > 100f && Re < 1000f) return (98.33f / Re) - (2778f / (Re * Re)) + 0.3644f;
        if (Re >= 1000f && Re < 5000f) return (148.62f / Re) - (4.75e4f / (Re * Re)) + 0.357f;
        return 0.44f;
    }

    // 4. 이론적 항력 기울기 계산 (tan_theory)
    public static float CalculateTanTheory(float U, float r, float cd)
    {
        if (U <= 0f || r <= 0f) return 0f;
        float theory = (3f * RHO_A * cd * (U * U)) / (8f * RHO_W * G * r);

        if (!Mathf.Approximately(U, lastU))
        {
            Debug.Log($"[IciclePhysics] 풍속 변화 감지! Calculated TanTheory: {theory:F4} (U: {U:F2}m/s)");
            lastU = U;
        }
        return theory;
    }

    // 5. 성장 속도(V) 자동 계산 (온도와 풍속의 상호작용)
    public static float ComputeVelocity(float tempCelsius, float windSpeed)
    {
        if (tempCelsius > 0f) return 0.001f;

        float vRef = 15f; // mm/s (기준 속도)
        float deltaT = Mathf.Clamp01((0f - tempCelsius) / 30f); // 0~-30도 정규화
        float tempFactor = 1f + deltaT;

        float uNorm = Mathf.Clamp01(windSpeed / U_MAX);
        float windFactor = 1f + uNorm;

        float v = vRef * tempFactor * windFactor;
        float clampedV = Mathf.Clamp(v, 0.1f, 50f);

        if (Mathf.Abs(clampedV - lastV) >= 0.5f)
        {
            Debug.Log($"[IciclePhysics] 속도 변화! v: {clampedV:F2} mm/s (T: {tempCelsius:F1}C)");
            lastV = clampedV;
        }
        return clampedV;
    }

    // 6. Beta: 반응 민감도 (v/L)
    public static float ComputeBeta(float vTipMmPerSec, float lengthScaleM)
    {
        float v = vTipMmPerSec / 1000f;               // m/s 변환
        float L = Mathf.Max(1e-6f, lengthScaleM);     // m
        float beta = v / L;

        if (!Mathf.Approximately(beta, lastBeta))
        {
            Debug.Log($"[IciclePhysics] Beta 변화! Computed Beta: {beta:F4}");
            lastBeta = beta;
        }
        return Mathf.Clamp(beta, 0f, 1f);
    }

    // 7. Kappa: 기하학적 비대칭 감도 (1/r)
    public static float ComputeKappa(float rLocalM)
    {
        float r_ref = 0.00244f;  //평균 반지름 2.44mm
        return r_ref / Mathf.Max(1e-4f, rLocalM);
    }

    //8. Aw: 수막 비대칭 지수(물 분포 기반)
    public static float CalculateAw(int dripVertexIndex, Vector3 windDir,
    Vector3[] vertices, float[] waterCoeff,
    Dictionary<int, List<int>> vertexNeighbors)
    {
        float waterLeeward = 0f;
        float waterWindward = 0f;

        if (vertexNeighbors.TryGetValue(dripVertexIndex, out var neighbors))
        {
            foreach (int nIdx in neighbors)
            {
                Vector3 dir = (vertices[nIdx] - vertices[dripVertexIndex]).normalized;
                float dot = Vector3.Dot(dir, windDir);

                // dot > 0 이면 바람이 불어가는 방향(풍하측)
                // 가중치를 dot의 크기에 비례하게 주어 방향성을 명확히 함
                if (dot > 0) waterLeeward += dot * waterCoeff[nIdx];
                else waterWindward += Mathf.Abs(dot) * waterCoeff[nIdx];
            }
        }

        float total = waterLeeward + waterWindward;
        if (total < 1e-6f) return 0f;

        // 결과가 양수면 바람 방향으로 더 많이 휨 (풍하측에 물이 많음)
        return (waterLeeward - waterWindward) / total;
    }


    // 9. Gamma
    public static float ComputeGamma(float tempCelsius, float waterValue)
    {
        float tempFactor = Mathf.Clamp01((0f - tempCelsius) / 30f);
        float waterFactor = Mathf.Clamp01(waterValue);

        float tauBase = 10f;
        float tau = tauBase / (1f + 1.5f * tempFactor + 0.5f * waterFactor);

        float gamma = 1f / tau;
        return Mathf.Clamp(gamma, 0f, 0.3f);
    }

    public static float CalculateIU(float windSpeed)
    {
        return Mathf.Clamp01(windSpeed / U_MAX);
    }

    // 10. 비대칭 성장항 통합 계산
    public static float ComputeAsymmetricGrowth(float kappa, float I_u, float Aw, float gamma)
    {
        return gamma * (kappa * I_u * Aw);
    }


}