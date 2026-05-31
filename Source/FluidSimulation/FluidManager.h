#pragma once

#include "CoreMinimal.h"
#include "FluidInfo.h"
#include "GameFramework/Actor.h"
#include "FluidManager.generated.h"

#define K_TAIT 7

class UFluidParticleSoA;
class UInstancedStaticMeshComponent;
class UStaticMesh;

struct FFluidNeighborInfo
{
	int Id = INDEX_NONE;
	FVector2D Delta = FVector2D::ZeroVector;
	float Distance = 0.0f;
	float PolyKernel = 0.0f;
	float DSpiky = 0.0f;
	float ViscosityWeight = 0.0f;
};

struct FFluidKernelCache
{
	float InfluenceRadius = 0.0f;
	float InfluenceRadiusSquared = 0.0f;
	float Poly2DCoef = 0.0f;
	float DSpiky2DCoef = 0.0f;
	float ViscosityDenominatorOffset = 0.0f;
};

UCLASS()
class FLUIDSIMULATION_API AFluidContainer : public AActor
{
	GENERATED_BODY()
	
public:
	AFluidContainer();
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	FVector ParticleToWorldPosition(const FVector2D& Position) const;

	virtual void Tick(float DeltaSeconds) override;
	void StepSimulation(float DeltaTime);
	void UpdateParticleVisuals();
public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid")
	float FixedDt = 0.003;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid")
	FVector2D Gravity = FVector2D(0,-9.81f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fluid", meta=(ClampMin="0.0", UIMin="0.0"))
	float GravityScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fluid", meta=(ClampMin="0.0", UIMin="0.0"))
	float MaxVelocity = 1.5f;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid")
	UFluidInfo* FluidInfo;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid")
	UFluidParticleSoA* Particles;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid")
	UStaticMesh* StaticMeshToInstance;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fluid")
	UInstancedStaticMeshComponent* InstancedStaticMesh;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fluid")
	FVector MeshScale = FVector(1);	

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fluid", meta=(ClampMin="0.0", UIMin="0.0"))
	float ScaleOffset = 1.0f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fluid")
	float WorldScale = 100.0f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fluid")
	float ParticleRadius = 0.18f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fluid")
	float ParticleMass = 0.0f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fluid")
	float WallBounce = 0.2f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fluid")
	float WallFriction = 0.2f;
	
	
	float GetContainerArea();
	
	float GetPolyKernel(const float CurrentDistance, const float InfluenceRadius);
	
	float GetPoly2DKernel(const float CurrentDistance, const float InfluenceRadius);
	
	float GetSpikyKernel(const float CurrentDistance, const float InfluenceRadius);
	float GetSpiky2DKernel(const float CurrentDistance, const float InfluenceRadius);
	
	float GetDSpiky(const float CurrentDistance, const float InfluenceRadius);
	float GetDSpiky2D(const float CurrentDistance, const float InfluenceRadius);

	float GetMass(float WantedRestDensity, float ContainerVolume, int NbrParticules, const float InfluenceRadius);
	
	float GetDensity(const TArray<FFluidNeighborInfo>& Neighbors, float Mass);
	
	float GetPression(float RestDensity, float WantedRestDensity);
	
	FVector2D GetPressureGradient(const TArray<FFluidNeighborInfo>& Neighbors, int ParticleId, float Mass, const TArray<float>& Densities, const TArray<float>& Pressions);
	
	FVector2D GetViscosity(const TArray<FFluidNeighborInfo>& Neighbors, FVector2D Velocity, float Mass, const TArray<float>& Densities);
	
	FVector2D GetAcceleration(FVector2D PressureGradient, float ViscosityStrength, FVector2D Viscosity, float Density);
	
	FVector2D GetExternalForces(float Density);

	void UpdateAutoParticleScale();
	
	UFUNCTION(BlueprintCallable, Category = "Fluid")
	bool GetDistanceWith(float& OutResult, const int& ParticleAId, const int& ParticleBId);
	
	void GetAllNeighborsIdOf(int IdToCheck, TArray<int>& OutResult);
	void GetAllNeighborsOf(int IdToCheck, TArray<FFluidNeighborInfo>& OutResult);

private:
	bool UpdateKernelCache();
	void EnsureSimulationBuffers(int ParticleCount);
	float GetCachedPoly2DKernel(float DistanceSquared) const;
	float GetCachedDSpiky2DKernel(float Distance) const;
	void BuildSpatialGrid();
	bool HasValidSpatialGrid() const;
	const FVector2D& GetParticleSimulationPosition(int ParticleId) const;
	FIntPoint GetSpatialCellCoords(const FVector2D& SimulationPosition) const;
	int GetSpatialCellIndex(const FIntPoint& CellCoords) const;
	bool IsSpatialCellInBounds(const FIntPoint& CellCoords) const;

	TArray<int> SpatialCellHeads;
	TArray<int> SpatialNextParticle;
	FIntPoint SpatialGridSize = FIntPoint(0, 0);
	FVector2D SpatialGridOrigin = FVector2D::ZeroVector;
	float SpatialCellSize = 0.0f;

	FFluidKernelCache KernelCache;
	TArray<TArray<FFluidNeighborInfo>> NeighborsByParticleBuffer;
	TArray<float> DensityBuffer;
	TArray<float> PressureBuffer;
};
