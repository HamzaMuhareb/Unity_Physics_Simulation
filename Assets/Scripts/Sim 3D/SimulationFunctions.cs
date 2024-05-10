using UnityEngine;
using Unity.Mathematics;
using UnityEngine.Rendering;

public static class SimulationFunctions
{
    static readonly float PI = 3.1415926f;
    public static int NumParticles;
    public static float Gravity;
    public static float DeltaTime;
    public static float CollisionDamping;
    public static float SmoothingRadius;
    public static float TargetDensity;
    public static float PressureMultiplier;
    public static float NearPressureMultiplier;
    public static float ViscosityStrength;
    public static Vector3 BoundsSize;
    public static Vector3 Centre;
    public static Matrix4x4 LocalToWorld;
    public static Matrix4x4 WorldToLocal;
    static readonly int3[] offsets3D = {
        new int3(-1, -1, -1),
        new int3(-1, -1, 0),
        new int3(-1, -1, 1),
        new int3(-1, 0, -1),
        new int3(-1, 0, 0),
        new int3(-1, 0, 1),
        new int3(-1, 1, -1),
        new int3(-1, 1, 0),
        new int3(-1, 1, 1),
        new int3(0, -1, -1),
        new int3(0, -1, 0),
        new int3(0, -1, 1),
        new int3(0, 0, -1),
        new int3(0, 0, 0),
        new int3(0, 0, 1),
        new int3(0, 1, -1),
        new int3(0, 1, 0),
        new int3(0, 1, 1),
        new int3(1, -1, -1),
        new int3(1, -1, 0),
        new int3(1, -1, 1),
        new int3(1, 0, -1),
        new int3(1, 0, 0),
        new int3(1, 0, 1),
        new int3(1, 1, -1),
        new int3(1, 1, 0),
        new int3(1, 1, 1)
    };

    // Constants used for hashing
    static readonly uint hashK1 = 15823;
    static readonly uint hashK2 = 9737333;
    static readonly uint hashK3 = 440817757;

    public static void SetSettings(float deltaTime, float gravity, float collisionDamping,
                                   float smoothingRadius, float targetDensity, float pressureMultiplier,
                                   float nearPressureMultiplier, float viscosityStrength, float3 boundsSize,
                                   float3 centre, float4x4 localToWorld, float4x4 worldToLocal)
    {
        DeltaTime = deltaTime;
        Gravity = gravity;
        CollisionDamping = collisionDamping;
        SmoothingRadius = smoothingRadius;
        TargetDensity = targetDensity;
        PressureMultiplier = pressureMultiplier;
        NearPressureMultiplier = nearPressureMultiplier;
        ViscosityStrength = viscosityStrength;
        BoundsSize = boundsSize;
        Centre = centre;
        LocalToWorld = localToWorld;
        WorldToLocal = worldToLocal;
    }

    public static int3 GetCell3D(float3 position, float radius)
    {
        return (int3)math.floor(position / radius);
    }

    public static uint HashCell3D(int3 cell)
    {
        return (uint)((cell.x * hashK1) + (cell.y * hashK2) + (cell.z * hashK3));
    }

    public static uint KeyFromHash(uint hash, uint tableSize)
    {
        return hash % tableSize;
    }

    public static float SmoothingKernelPoly6(float dst, float radius)
    {
        if (dst < radius)
        {
            float scale = 315 / (64 * PI * math.pow(math.abs(radius), 9));
            float v = radius * radius - dst * dst;
            return v * v * v * scale;
        }
        return 0;
    }

    public static float SpikyKernelPow3(float dst, float radius)
    {
        if (dst < radius)
        {
            float scale = 15 / (PI * math.pow(radius, 6));
            float v = radius - dst;
            return v * v * v * scale;
        }
        return 0;
    }

    public static float SpikyKernelPow2(float dst, float radius)
    {
        if (dst < radius)
        {
            float scale = 15 / (2 * PI * math.pow(radius, 5));
            float v = radius - dst;
            return v * v * scale;
        }
        return 0;
    }

    public static float DerivativeSpikyPow3(float dst, float radius)
    {
        if (dst <= radius)
        {
            float scale = 45 / (math.pow(radius, 6) * PI);
            float v = radius - dst;
            return -v * v * scale;
        }
        return 0;
    }

    public static float DerivativeSpikyPow2(float dst, float radius)
    {
        if (dst <= radius)
        {
            float scale = 15 / (math.pow(radius, 5) * PI);
            float v = radius - dst;
            return -v * scale;
        }
        return 0;
    }

    public static float DensityKernel(float dst, float radius)
    {
        return SpikyKernelPow2(dst, radius);
    }

    public static float NearDensityKernel(float dst, float radius)
    {
        return SpikyKernelPow3(dst, radius);
    }

    public static float DensityDerivative(float dst, float radius)
    {
        return DerivativeSpikyPow2(dst, radius);
    }

    public static float NearDensityDerivative(float dst, float radius)
    {
        return DerivativeSpikyPow3(dst, radius);
    }

    public static float PressureFromDensity(float density,float TargetDensity , float PressureMultiplier)
    {
        return (density - TargetDensity) * PressureMultiplier;
    }

    public static float NearPressureFromDensity(float nearDensity, float NearPressureMultiplier)
    {
        return nearDensity * NearPressureMultiplier;
    }

    public static void ExternalForces(ComputeBuffer PositionsBuffer, ComputeBuffer VelocitiesBuffer, int NumParticles, float Gravity, float DeltaTime)
    {
        Vector3[] positions = new Vector3[PositionsBuffer.count];
        Vector3[] velocities = new Vector3[VelocitiesBuffer.count];
        PositionsBuffer.GetData(positions);
        VelocitiesBuffer.GetData(velocities);

        for (int i = 0; i < NumParticles; i++)
        {
            velocities[i] += Vector3.up * Gravity * DeltaTime;
            positions[i] += velocities[i] * DeltaTime;
        }

        PositionsBuffer.SetData(positions);
        VelocitiesBuffer.SetData(velocities);
    }

    public static void UpdateSpatialHash(ComputeBuffer PositionsBuffer, ComputeBuffer PredictedPositionsBuffer, ComputeBuffer SpatialIndicesBuffer, ComputeBuffer SpatialOffsetsBuffer, int NumParticles, float SmoothingRadius)
    {
        Vector3[] predictedPositions = new Vector3[PredictedPositionsBuffer.count];
        PredictedPositionsBuffer.GetData(predictedPositions);
        int3[] spatialIndices = new int3[NumParticles];
        uint[] spatialOffsets = new uint[NumParticles];

        for (int i = 0; i < NumParticles; i++)
        {
            int3 cell = GetCell3D(predictedPositions[i], SmoothingRadius);
            uint hash = HashCell3D(cell);
            uint key = KeyFromHash(hash, (uint)NumParticles);
            spatialIndices[i] = new int3(i, (int)hash, (int)key);
            spatialOffsets[i] = (uint)NumParticles;
        }

        SpatialIndicesBuffer.SetData(spatialIndices);
        SpatialOffsetsBuffer.SetData(spatialOffsets);
    }

    public static void CalculateDensities(ComputeBuffer PredictedPositionsBuffer, ComputeBuffer DensitiesBuffer, ComputeBuffer SpatialIndicesBuffer, ComputeBuffer SpatialOffsetsBuffer, int NumParticles, float SmoothingRadius)
    {
        Vector3[] predictedPositions = new Vector3[PredictedPositionsBuffer.count];
        PredictedPositionsBuffer.GetData(predictedPositions);
        Vector2[] densities = new Vector2[NumParticles];

        uint[] offsetsData = new uint[NumParticles];
        SpatialOffsetsBuffer.GetData(offsetsData);
        int3[] indexDataArray = new int3[NumParticles];
        SpatialIndicesBuffer.GetData(indexDataArray);
        for (int i = 0; i < NumParticles; i++)
        {
            Vector3 pos = predictedPositions[i];
            int3 originCell = GetCell3D(pos, SmoothingRadius);
            float sqrRadius = SmoothingRadius * SmoothingRadius;
            float density = 0;
            float nearDensity = 0;
            for (int j = 0; j < 27; j++)
            {
                uint hash = HashCell3D(originCell + offsets3D[j]);
                uint key = KeyFromHash(hash, (uint)NumParticles);
                uint currIndex = offsetsData[key];

                while (currIndex < NumParticles)
                {
                    int3 indexData = indexDataArray[currIndex];
                    currIndex++;
                    if (indexData.z != key) break;
                    if (indexData.y != hash) continue;

                    int NeighborIndex = indexData.x;
                    if (NeighborIndex == i) continue;

                    Vector3 NeighborPos = predictedPositions[NeighborIndex];
                    float sqrDstToNeighbor = (NeighborPos - pos).sqrMagnitude;

                    if (sqrDstToNeighbor > sqrRadius) continue;

                    float dst = Mathf.Sqrt(sqrDstToNeighbor);
                    density += DensityKernel(dst, SmoothingRadius);
                    nearDensity += NearDensityKernel(dst, SmoothingRadius);
                }
            }

            densities[i] = new Vector2(density, nearDensity);
        }

        DensitiesBuffer.SetData(densities);
    }

    public static void CalculatePressureForce(ComputeBuffer PredictedPositionsBuffer, ComputeBuffer VelocitiesBuffer, ComputeBuffer DensitiesBuffer, ComputeBuffer SpatialIndicesBuffer, ComputeBuffer SpatialOffsetsBuffer, int NumParticles, float SmoothingRadius, float TargetDensity, float PressureMultiplier, float NearPressureMultiplier,float DeltaTime)
    {
        Vector3[] predictedPositions = new Vector3[PredictedPositionsBuffer.count];
        PredictedPositionsBuffer.GetData(predictedPositions);
        Vector2[] densities = new Vector2[DensitiesBuffer.count];
        DensitiesBuffer.GetData(densities);
        Vector3[] velocities = new Vector3[VelocitiesBuffer.count];
        VelocitiesBuffer.GetData(velocities);
        uint[] offsetsData = new uint[NumParticles];
        SpatialOffsetsBuffer.GetData(offsetsData);
        int3[] indexDataArray = new int3[NumParticles];
        SpatialIndicesBuffer.GetData(indexDataArray);

        // Debug.Log(predictedPositions[100]);
        // Debug.Log(densities[100]);
        // Debug.Log(velocities[100]);
        // Debug.Log(offsetsData[100]);
        // Debug.Log(indexDataArray[100]);

        for (int i = 0; i < NumParticles; i++)
        {
            float density = densities[i].x;
            float nearDensity = densities[i].y;
            float pressure = PressureFromDensity(density , TargetDensity ,PressureMultiplier);
            float nearPressure = NearPressureFromDensity(nearDensity,NearPressureMultiplier);
            Vector3 pressureForce = Vector3.zero;

            Vector3 pos = predictedPositions[i];
            int3 originCell = GetCell3D(pos, SmoothingRadius);
            float sqrRadius = SmoothingRadius * SmoothingRadius;

            for (int j = 0; j < 27; j++)
            {
                uint hash = HashCell3D(originCell + offsets3D[j]);
                uint key = KeyFromHash(hash, (uint)NumParticles);
                uint currIndex = offsetsData[key];

                while (currIndex < NumParticles)
                {
                    int3 indexData = indexDataArray[currIndex];
                    currIndex++;
                    if (indexData.z != key) break;
                    if (indexData.y != hash) continue;

                    int NeighborIndex = indexData.x;
                    if (NeighborIndex == i) continue;

                    Vector3 NeighborPos = predictedPositions[NeighborIndex];
                    float sqrDstToNeighbor = (NeighborPos - pos).sqrMagnitude;

                    if (sqrDstToNeighbor > sqrRadius) continue;

                    float densityNeighbor = densities[NeighborIndex].x;
                    float nearDensityNeighbor = densities[NeighborIndex].y;
                    float NeighborPressure = PressureFromDensity(densityNeighbor,TargetDensity ,PressureMultiplier);
                    float NeighborPressureNear = NearPressureFromDensity(nearDensityNeighbor ,NearPressureMultiplier);

                    float sharedPressure = (pressure + NeighborPressure) / 2;
                    float sharedNearPressure = (nearPressure + NeighborPressureNear) / 2;

                    float dst = Mathf.Sqrt(sqrDstToNeighbor);
                    Vector3 dir = dst > 0 ? (NeighborPos - pos) / dst : Vector3.up;

                    pressureForce += dir * DensityDerivative(dst, SmoothingRadius) * sharedPressure / densityNeighbor;
                    pressureForce += dir * NearDensityDerivative(dst, SmoothingRadius) * sharedNearPressure / nearDensityNeighbor;
                }
            }

            velocities[i] += pressureForce / density * DeltaTime;
        }

        // VelocitiesBuffer.SetData(velocities);
    }

    public static void CalculateViscosity(ComputeBuffer PredictedPositionsBuffer, ComputeBuffer VelocitiesBuffer, ComputeBuffer SpatialIndicesBuffer, ComputeBuffer SpatialOffsetsBuffer, int NumParticles, float SmoothingRadius, float ViscosityStrength)
    {
        Vector3[] predictedPositions = new Vector3[PredictedPositionsBuffer.count];
        PredictedPositionsBuffer.GetData(predictedPositions);
        Vector3[] velocities = new Vector3[VelocitiesBuffer.count];
        VelocitiesBuffer.GetData(velocities);


        uint[] offsetsData = new uint[NumParticles];
        SpatialOffsetsBuffer.GetData(offsetsData);
        int3[] indexDataArray = new int3[NumParticles];
        SpatialIndicesBuffer.GetData(indexDataArray);

        for (int i = 0; i < NumParticles; i++)
        {
            Vector3 pos = predictedPositions[i];
            int3 originCell = GetCell3D(pos, SmoothingRadius);
            float sqrRadius = SmoothingRadius * SmoothingRadius;
            Vector3 viscosityForce = Vector3.zero;
            Vector3 velocity = velocities[i];

            for (int j = 0; j < 27; j++)
            {
                uint hash = HashCell3D(originCell + offsets3D[j]);
                uint key = KeyFromHash(hash, (uint)NumParticles);
                uint currIndex = offsetsData[key];

                while (currIndex < NumParticles)
                {
                    int3 indexData = indexDataArray[currIndex];
                    currIndex++;
                    if (indexData.z != key) break;
                    if (indexData.y != hash) continue;

                    int NeighborIndex = indexData.x;
                    if (NeighborIndex == i) continue;

                    Vector3 NeighborPos = predictedPositions[NeighborIndex];
                    float sqrDstToNeighbor = (NeighborPos - pos).sqrMagnitude;

                    if (sqrDstToNeighbor > sqrRadius) continue;

                    viscosityForce += (velocities[NeighborIndex] - velocity) * SmoothingKernelPoly6(Mathf.Sqrt(sqrDstToNeighbor), SmoothingRadius);
                }
            }

            velocities[i] += viscosityForce * ViscosityStrength * DeltaTime;
        }

        VelocitiesBuffer.SetData(velocities);
    }

    public static void UpdatePositions(ComputeBuffer PositionsBuffer, ComputeBuffer PredictedPositionsBuffer, ComputeBuffer VelocitiesBuffer, int NumParticles, float DeltaTime, float CollisionDamping)
    {
        Vector3[] positions = new Vector3[PositionsBuffer.count];
        Vector3[] velocities = new Vector3[VelocitiesBuffer.count];
        PositionsBuffer.GetData(positions);
        VelocitiesBuffer.GetData(velocities);

        for (int i = 0; i < NumParticles; i++)
        {
            positions[i] += velocities[i] * DeltaTime;
        }

        PositionsBuffer.SetData(positions);
    }
    public static void ResolveCollisions(float CollisionDamping, ComputeBuffer PositionsBuffer, ComputeBuffer VelocitiesBuffer, int NumParticles)
    {
        Vector3[] positionsData = new Vector3[PositionsBuffer.count];
        Vector3[] velocitiesData = new Vector3[VelocitiesBuffer.count];
        PositionsBuffer.GetData(positionsData);
        VelocitiesBuffer.GetData(velocitiesData);
        for (int particleIndex = 0; particleIndex < NumParticles; particleIndex++)
        {
            Vector3 posLocal = WorldToLocal.MultiplyPoint(positionsData[particleIndex]);
            Vector3 velocityLocal = WorldToLocal.MultiplyVector(velocitiesData[particleIndex]);

            // Calculate distance from box on each axis (negative values are inside box)
            Vector3 halfSize = BoundsSize * 0.5f;
            Vector3 absPosLocal = new Vector3(Mathf.Abs(posLocal.x), Mathf.Abs(posLocal.y), Mathf.Abs(posLocal.z));
            Vector3 edgeDst = halfSize - absPosLocal;

            // Resolve collisions
            if (edgeDst.x <= 0)
            {
                posLocal.x = halfSize.x * Mathf.Sign(posLocal.x);
                velocityLocal.x *= -1 * CollisionDamping;
            }
            if (edgeDst.y <= 0)
            {
                posLocal.y = halfSize.y * Mathf.Sign(posLocal.y);
                velocityLocal.y *= -1 * CollisionDamping;
            }
            if (edgeDst.z <= 0)
            {
                posLocal.z = halfSize.z * Mathf.Sign(posLocal.z);
                velocityLocal.z *= -1 * CollisionDamping;
            }
            positionsData[particleIndex] = LocalToWorld.MultiplyPoint(posLocal);
            velocitiesData[particleIndex] = LocalToWorld.MultiplyPoint(velocityLocal);
        }
        // Set the modified data back to the buffer
        PositionsBuffer.SetData(positionsData);
        VelocitiesBuffer.SetData(velocitiesData);
    }

}
