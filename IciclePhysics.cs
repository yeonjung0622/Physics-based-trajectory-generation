using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class IciclePhysics 
{
    private const float RHO_A = 1.2f;
    private const float RHO_W = 1000f;
    private const float G = 9.8f;


    private static float lastU = -999f;
    private static float lastBeta = -1f; 
    public static float CalculateEta(float tempCelsius)
    {
        float T = tempCelsius + 273.15f;
        return 1.716e-5f * Mathf.Pow(T / 273.15f, 1.5f) * (273.15f + 110.4f) / (T + 110.4f);
    }

    public static float CalculateReynolds(float U, float r, float eta)
    {
        if (eta <= 0f) return 0f;
        return (2f * RHO_A * r * U) / eta;
    }

    public static float CalculateCd(float Re)
    {
        if (Re > 100f && Re < 1000f) return (98.33f / Re) - (2778f / (Re * Re)) + 0.3644f;
        if (Re >= 1000f && Re < 5000f) return (148.62f / Re) - (4.75e4f / (Re * Re)) + 0.357f;
        return 0.44f;
    }

    public static float CalculateTanTheory(float U, float r, float cd)
    {
        if (U <= 0f || r <= 0f) return 0f;
        float theory = (3f * RHO_A * cd * (U * U)) / (8f * RHO_W * G * r);
        if (!Mathf.Approximately(U, lastU))
        {
            Debug.Log($"[IciclePhysics] 풍속 변화 감지! Calculated TanTheory: {theory:F4} (풍속 U: {U:F2}, 반지름 r: {r:F4}, 항력계수 cd: {cd:F4})");
            lastU = U; 
        }
            return theory; 
    }


    // Beta: 성장 속도와 마디 길이에 따른 반응성 (단위: 1/s)
    public static float ComputeBeta(float vTipMmPerSec, float lengthScaleM)
    {
        float v = vTipMmPerSec / 1000f;               // m/s
        float L = Mathf.Max(1e-6f, lengthScaleM);     // m
        float beta = v / L; //1/s

        if (!Mathf.Approximately(beta, lastBeta))
        {
            Debug.Log($"[IciclePhysics] Beta 값 변화 감지! Computed Beta: {beta:F4} (성장속도 v: {v:F4}m/s, 마디길이 L: {L:F4}m)");
            lastBeta = beta;
        }

        return beta; 
    }

    // Kappa: 반지름에 반비례하는 비대칭 감도 (얇을수록 더 잘 휘어짐)
    public static float ComputeKappa(float rLocalM)
    {
        return 1f / Mathf.Max(1e-6f, rLocalM);    // 1/m
    }

    // Gamma: 바람 방향과 중력 방향의 관계, 풍속, 비대칭 지수를 통합한 누적 강도
    public static float ComputeGamma(Vector3 windDirWorld, float windSpeed, Vector3 gravityDirWorld, float Aw01, float Uref = 15f)
    {
        Vector3 g = gravityDirWorld.sqrMagnitude > 1e-8f ? gravityDirWorld.normalized : Vector3.down;
        Vector3 w = windDirWorld.sqrMagnitude > 1e-8f ? windDirWorld.normalized : Vector3.right;

        // 중력에 수직인 바람 성분 비율(0..1) - 실제로 굽힘을 만드는 성분
        float perpRatio = Vector3.ProjectOnPlane(w, g).magnitude;

        float uFactor = Mathf.Clamp01(windSpeed / Mathf.Max(1e-6f, Uref));
        float aFactor = Mathf.Clamp01(Aw01);

        return Mathf.Clamp01(perpRatio * uFactor * aFactor);
    }
}
