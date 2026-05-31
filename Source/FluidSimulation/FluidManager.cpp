// Fill out your copyright notice in the Description page of Project Settings.


#include "FluidManager.h"
#include "FluidParticleSoA.h"
#include "Async/ParallelFor.h"
#include "HeadMountedDisplayTypes.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/Engine.h"
#include "PostProcess/PostProcessMaterialInputs.h"

AFluidContainer::AFluidContainer()
{
	PrimaryActorTick.bCanEverTick = true;
	InstancedStaticMesh = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("InstancedMesh"));
	InstancedStaticMesh->SetCollisionProfileName(TEXT("OverlapAllDynamic"));
	RootComponent = InstancedStaticMesh;
}

void AFluidContainer::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	UpdateAutoParticleScale();
	if (IsValid(FluidInfo) && FluidInfo->NbOfParticles > 0)
	{
		ParticleMass = GetMass(FluidInfo->WantedRestDensity, GetContainerArea(), FluidInfo->NbOfParticles, FluidInfo->InfluenceRadius);
	}
}

void AFluidContainer::BeginPlay()
{
	Super::BeginPlay();
	if (IsValid(FluidInfo))
	{
		Particles = NewObject<UFluidParticleSoA>(this);
		FVector2D SafeContainerBounds = FluidInfo->ContainerBounds;
		// pour faire en sorte que ca spawn pas en touchant directement les mur
		SafeContainerBounds.X -= (ParticleRadius * 2);
		SafeContainerBounds.Y -= (ParticleRadius * 2);
		SafeContainerBounds.X *= 0.8;
		SafeContainerBounds.Y *= 0.8;
		
			FVector2D StartPosition = -SafeContainerBounds/ 2;
			Particles->Init(FluidInfo->NbOfParticles, StartPosition, SafeContainerBounds, FluidInfo->InfluenceRadius / 2);
			const int RequestedParticleCount = FluidInfo->NbOfParticles;
			const int SpawnedParticleCount = Particles->Position.Num();
			if (SpawnedParticleCount > 0 && SpawnedParticleCount < RequestedParticleCount && GEngine)
			{
				const FString Message = FString::Printf(
					TEXT("Fluid: %d particules demandees, %d creees seulement. Container trop petit / espacement trop grand."),
					RequestedParticleCount,
					SpawnedParticleCount);
				GEngine->AddOnScreenDebugMessage(INDEX_NONE, 8.0f, FColor::Orange, Message);
			}
			if (Particles->Position.IsEmpty())
			{
				UE_LOG(LogTemp, Error, TEXT("No fluid particles were spawned"));
				if (GEngine)
				{
					GEngine->AddOnScreenDebugMessage(INDEX_NONE, 8.0f, FColor::Red, TEXT("Fluid: aucune particule creee. Verifie NbOfParticles, ContainerBounds et InfluenceRadius."));
				}
				return;
			}
		ParticleMass = GetMass(FluidInfo->WantedRestDensity, GetContainerArea(), Particles->Id.Num(), FluidInfo->InfluenceRadius);
		
		
		UpdateAutoParticleScale();
		FTransform InstanceTransform;
		InstanceTransform.SetRotation(FQuat::Identity);
		InstanceTransform.SetScale3D(MeshScale);
		for (const FVector2D& ParticlePosition : Particles->Position)
		{
			InstanceTransform.SetLocation(ParticleToWorldPosition(ParticlePosition));
			InstancedStaticMesh->AddInstance(InstanceTransform, true);
		}
	}
}

FVector AFluidContainer::ParticleToWorldPosition(const FVector2D& Position) const
{
	return GetActorLocation() + FVector(Position.X * WorldScale, 0.0f, Position.Y * WorldScale);
}

bool AFluidContainer::UpdateKernelCache()
{
	if (!IsValid(FluidInfo) || FluidInfo->InfluenceRadius <= 0.0f)
	{
		KernelCache = FFluidKernelCache{};
		return false;
	}

	const float H = FluidInfo->InfluenceRadius;
	const float H2 = H * H;
	const float H4 = H2 * H2;
	const float H5 = H4 * H;
	const float H8 = H4 * H4;
	const float ViscosityFactor = FluidInfo->ViscosityFactor > 0.0f ? FluidInfo->ViscosityFactor : 0.01f;

	KernelCache.InfluenceRadius = H;
	KernelCache.InfluenceRadiusSquared = H2;
	KernelCache.Poly2DCoef = 4.0f / (PI * H8);
	KernelCache.DSpiky2DCoef = -30.0f / (PI * H5);
	KernelCache.ViscosityDenominatorOffset = ViscosityFactor * H2;
	return true;
}

void AFluidContainer::EnsureSimulationBuffers(int ParticleCount)
{
	NeighborsByParticleBuffer.SetNum(ParticleCount);
	DensityBuffer.SetNumUninitialized(ParticleCount);
	PressureBuffer.SetNumUninitialized(ParticleCount);
}

float AFluidContainer::GetCachedPoly2DKernel(float DistanceSquared) const
{
	const float Remaining = KernelCache.InfluenceRadiusSquared - DistanceSquared;
	return Remaining > 0.0f ? KernelCache.Poly2DCoef * Remaining * Remaining * Remaining : 0.0f;
}

float AFluidContainer::GetCachedDSpiky2DKernel(float Distance) const
{
	const float Remaining = KernelCache.InfluenceRadius - Distance;
	return Remaining > 0.0f ? KernelCache.DSpiky2DCoef * Remaining * Remaining : 0.0f;
}

bool AFluidContainer::HasValidSpatialGrid() const
{
	return SpatialCellSize > 0.0f
		&& SpatialGridSize.X > 0
		&& SpatialGridSize.Y > 0
		&& SpatialGridSize.Z > 0
		&& SpatialCellHeads.Num() == SpatialGridSize.X * SpatialGridSize.Y * SpatialGridSize.Z
		&& IsValid(Particles)
		&& SpatialNextParticle.Num() == Particles->Position.Num();
}

FVector AFluidContainer::GetParticleSimulationPosition(int ParticleId) const
{
	const FVector2D Position = Particles->Position[ParticleId];
	// The neighbor grid is 3D internally. The current solver is 2D, so all
	// particles live on one Z slice; moving to 3D only needs this mapping to
	// return the particle's real Z coordinate and the grid depth to exceed one.
	return FVector(Position.X, Position.Y, 0.0f);
}

FIntVector AFluidContainer::GetSpatialCellCoords(const FVector& SimulationPosition) const
{
	if (SpatialCellSize <= 0.0f || SpatialGridSize.X <= 0 || SpatialGridSize.Y <= 0 || SpatialGridSize.Z <= 0)
	{
		return FIntVector::ZeroValue;
	}

	const FVector LocalPosition = (SimulationPosition - SpatialGridOrigin) / SpatialCellSize;
	return FIntVector(
		FMath::Clamp(FMath::FloorToInt(LocalPosition.X), 0, SpatialGridSize.X - 1),
		FMath::Clamp(FMath::FloorToInt(LocalPosition.Y), 0, SpatialGridSize.Y - 1),
		FMath::Clamp(FMath::FloorToInt(LocalPosition.Z), 0, SpatialGridSize.Z - 1));
}

int AFluidContainer::GetSpatialCellIndex(const FIntVector& CellCoords) const
{
	return CellCoords.X
		+ CellCoords.Y * SpatialGridSize.X
		+ CellCoords.Z * SpatialGridSize.X * SpatialGridSize.Y;
}

bool AFluidContainer::IsSpatialCellInBounds(const FIntVector& CellCoords) const
{
	return CellCoords.X >= 0
		&& CellCoords.Y >= 0
		&& CellCoords.Z >= 0
		&& CellCoords.X < SpatialGridSize.X
		&& CellCoords.Y < SpatialGridSize.Y
		&& CellCoords.Z < SpatialGridSize.Z;
}

void AFluidContainer::BuildSpatialGrid()
{
	if (!IsValid(FluidInfo) || !IsValid(Particles) || FluidInfo->InfluenceRadius <= 0.0f)
	{
		SpatialCellHeads.Reset();
		SpatialNextParticle.Reset();
		SpatialGridSize = FIntVector::ZeroValue;
		SpatialCellSize = 0.0f;
		return;
	}

	SpatialCellSize = FluidInfo->InfluenceRadius;
	SpatialGridOrigin = FVector(-FluidInfo->ContainerBounds.X * 0.5f, -FluidInfo->ContainerBounds.Y * 0.5f, 0.0f);

	const int GridWidth = FMath::Max(1, FMath::CeilToInt(FluidInfo->ContainerBounds.X / SpatialCellSize));
	const int GridHeight = FMath::Max(1, FMath::CeilToInt(FluidInfo->ContainerBounds.Y / SpatialCellSize));
	const int GridDepth = 1;
	SpatialGridSize = FIntVector(GridWidth, GridHeight, GridDepth);

	const int CellCount = SpatialGridSize.X * SpatialGridSize.Y * SpatialGridSize.Z;
	SpatialCellHeads.Init(INDEX_NONE, CellCount);
	SpatialNextParticle.SetNumUninitialized(Particles->Position.Num());

	for (int ParticleId = 0; ParticleId < Particles->Position.Num(); ++ParticleId)
	{
		const FIntVector CellCoords = GetSpatialCellCoords(GetParticleSimulationPosition(ParticleId));
		const int CellIndex = GetSpatialCellIndex(CellCoords);
		SpatialNextParticle[ParticleId] = SpatialCellHeads[CellIndex];
		SpatialCellHeads[CellIndex] = ParticleId;
	}
}

void AFluidContainer::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!IsValid(FluidInfo) || !IsValid(Particles))
	{
		UE_LOG(LogTemp, Error, TEXT("FluidInfo / Particles isn't valid"));
		return;
	}
	DrawDebugBox(GetWorld(), GetActorLocation(), FVector(FluidInfo->ContainerBounds.X * WorldScale / 2, 5, FluidInfo->ContainerBounds.Y * WorldScale / 2), FColor::Red, false, -1, 0, 10);
	
	
	float AccumulatedTime = 0.0f;
	while (AccumulatedTime < DeltaSeconds)
	{
		float DeltaTimeToUse = AccumulatedTime + FixedDt < DeltaSeconds ? FixedDt : DeltaSeconds - AccumulatedTime;
		AccumulatedTime += DeltaTimeToUse;
		StepSimulation(DeltaTimeToUse);
	}
	UpdateParticleVisuals();

}

void AFluidContainer::UpdateParticleVisuals()
{
	if (!IsValid(Particles) || !IsValid(InstancedStaticMesh))
	{
		return;
	}

	const int ParticleCount = Particles->Position.Num();
	for (int ParticleId = 0; ParticleId < ParticleCount; ++ParticleId)
	{
		const bool bIsLastInstance = ParticleId == ParticleCount - 1;
		FTransform T(FRotator::ZeroRotator, ParticleToWorldPosition(Particles->Position[ParticleId]), MeshScale);
		InstancedStaticMesh->UpdateInstanceTransform(ParticleId, T, true, bIsLastInstance, true);
	}
}

void AFluidContainer::StepSimulation(float DeltaTime)
{
	const int ParticleCount = Particles->Id.Num();
	if (ParticleCount == 0)
	{
		return;
	}

	ParticleMass = GetMass(FluidInfo->WantedRestDensity, GetContainerArea(), ParticleCount, FluidInfo->InfluenceRadius);
	const float Mass = ParticleMass;
	if (FMath::IsNearlyZero(Mass))
	{
		return;
	}
	if (!UpdateKernelCache())
	{
		return;
	}
	BuildSpatialGrid();
	if (!HasValidSpatialGrid())
	{
		return;
	}

	EnsureSimulationBuffers(ParticleCount);

	constexpr int32 ParallelParticleThreshold = 256;
	const bool bUseParallelFor = ParticleCount >= ParallelParticleThreshold;

	auto ComputeDensityAndPressure = [this, Mass](int32 ParticleIndex)
	{
		const int ParticleId = Particles->Id[ParticleIndex];
		GetAllNeighborsOf(ParticleId, NeighborsByParticleBuffer[ParticleId]);
		DensityBuffer[ParticleId] = GetDensity(NeighborsByParticleBuffer[ParticleId], Mass);
		PressureBuffer[ParticleId] = GetPression(DensityBuffer[ParticleId], FluidInfo->WantedRestDensity);
	};

	if (bUseParallelFor)
	{
		ParallelFor(ParticleCount, ComputeDensityAndPressure);
	}
	else
	{
		for (int32 ParticleIndex = 0; ParticleIndex < ParticleCount; ++ParticleIndex)
		{
			ComputeDensityAndPressure(ParticleIndex);
		}
	}

	auto ComputeAcceleration = [this, Mass](int32 ParticleIndex)
	{
		const int ParticleId = Particles->Id[ParticleIndex];
		const FVector2D PressionGradient = GetPressureGradient(NeighborsByParticleBuffer[ParticleId], ParticleId, Mass, DensityBuffer, PressureBuffer);
		const FVector2D Viscosity = GetViscosity(NeighborsByParticleBuffer[ParticleId], Particles->Velocity[ParticleId], Mass, DensityBuffer);
		Particles->AccelerationToUse[ParticleId] = GetAcceleration(PressionGradient, FluidInfo->ViscosityStrength, Viscosity, DensityBuffer[ParticleId]);
	};

	if (bUseParallelFor)
	{
		ParallelFor(ParticleCount, ComputeAcceleration);
	}
	else
	{
		for (int32 ParticleIndex = 0; ParticleIndex < ParticleCount; ++ParticleIndex)
		{
			ComputeAcceleration(ParticleIndex);
		}
	}

	const float MaxVelocitySquared = FMath::Square(MaxVelocity);
	const float MinX = -1 * FluidInfo->ContainerBounds.X / 2 + ParticleRadius;
	const float MinY = -1 * FluidInfo->ContainerBounds.Y / 2 + ParticleRadius;
	const float MaxX = FluidInfo->ContainerBounds.X / 2 - ParticleRadius;
	const float MaxY = FluidInfo->ContainerBounds.Y / 2 - ParticleRadius;
	const float Bounce = WallBounce * -1.0f;
	const float Friction = WallFriction;

	auto IntegrateParticle = [this, DeltaTime, MaxVelocitySquared, MinX, MinY, MaxX, MaxY, Bounce, Friction](int32 ParticleIndex)
	{
		const int ParticleId = Particles->Id[ParticleIndex];
		Particles->Acceleration[ParticleId] = Particles->AccelerationToUse[ParticleId];
		
		Particles->Velocity[ParticleId] = Particles->Velocity[ParticleId] + DeltaTime * Particles->Acceleration[ParticleId];
		if (MaxVelocity > 0.0f && Particles->Velocity[ParticleId].SizeSquared() > MaxVelocitySquared)
		{
			Particles->Velocity[ParticleId] = Particles->Velocity[ParticleId].GetSafeNormal() * MaxVelocity;
		}
		Particles->Position[ParticleId] = Particles->Position[ParticleId] + DeltaTime * Particles->Velocity[ParticleId];
		
		if (Particles->Position[ParticleId].X < MinX)
		{
			Particles->Position[ParticleId].X = MinX;
			if (Particles->Velocity[ParticleId].X < 0)
			{
				Particles->Velocity[ParticleId].X *=  Bounce;
			}
			Particles->Velocity[ParticleId].Y *= Friction;
		}
		
		if (Particles->Position[ParticleId].X > MaxX)
		{
			Particles->Position[ParticleId].X = MaxX;
			if (Particles->Velocity[ParticleId].X > 0)
			{
				Particles->Velocity[ParticleId].X *= Bounce;
			}
			Particles->Velocity[ParticleId].Y *= Friction;
		}
		
		if (Particles->Position[ParticleId].Y < MinY)
		{
			Particles->Position[ParticleId].Y = MinY;
			if (Particles->Velocity[ParticleId].Y < 0)
			{
				Particles->Velocity[ParticleId].Y *= Bounce;
			}
			Particles->Velocity[ParticleId].X *= Friction;
			
		}
		
		if (Particles->Position[ParticleId].Y > MaxY)
		{
			Particles->Position[ParticleId].Y = MaxY;
			if (Particles->Velocity[ParticleId].Y > 0)
			{
				Particles->Velocity[ParticleId].Y *= Bounce;
			}
			Particles->Velocity[ParticleId].X *= Friction;
		}

		if (MaxVelocity > 0.0f && Particles->Velocity[ParticleId].SizeSquared() > MaxVelocitySquared)
		{
			Particles->Velocity[ParticleId] = Particles->Velocity[ParticleId].GetSafeNormal() * MaxVelocity;
		}
		
		
		
		// Particles->Position[ParticleId].X = FMath::Max(-1 * FluidInfo->ContainerBounds.X / 2, Particles->Position[ParticleId].X);
		// Particles->Position[ParticleId].Y = FMath::Max(-1 * FluidInfo->ContainerBounds.Y / 2, Particles->Position[ParticleId].Y);
		//
		// Particles->Position[ParticleId].X = FMath::Min(FluidInfo->ContainerBounds.X / 2, Particles->Position[ParticleId].X);
		// Particles->Position[ParticleId].Y = FMath::Min(FluidInfo->ContainerBounds.Y / 2, Particles->Position[ParticleId].Y);		
		
	};

	if (bUseParallelFor)
	{
		ParallelFor(ParticleCount, IntegrateParticle);
	}
	else
	{
		for (int32 ParticleIndex = 0; ParticleIndex < ParticleCount; ++ParticleIndex)
		{
			IntegrateParticle(ParticleIndex);
		}
	}
	
}

float AFluidContainer::GetContainerArea()
{
	return FluidInfo->ContainerBounds.X * FluidInfo->ContainerBounds.Y;
}

float AFluidContainer::GetPolyKernel(const float CurrentDistance, const float InfluenceRadius)
{
	if (FMath::IsNearlyZero(InfluenceRadius))
	{
		UE_LOG(LogTemp, Error, TEXT("GetPolyKernel : InfluenceRadius is zero"));
		return 0;
	}
	
	auto ToDivide = (64*PI*(FMath::Pow(InfluenceRadius,9)));
	auto ToMultiply = FMath::Pow((InfluenceRadius*InfluenceRadius - CurrentDistance*CurrentDistance),3);
	
	return 315/ToDivide * ToMultiply;
}

float AFluidContainer::GetSpikyKernel(const float CurrentDistance, const float InfluenceRadius)
{
	auto ToDivide = PI*(FMath::Pow(InfluenceRadius,6));
	if (FMath::IsNearlyZero(InfluenceRadius))
	{
		UE_LOG(LogTemp, Error, TEXT("GetSpikyKernel : InfluenceRadius is zero"));
		return 0;
	}
	auto ToMultiply = FMath::Pow((InfluenceRadius - CurrentDistance),3);

	return 15/ToDivide * ToMultiply;
}

float AFluidContainer::GetDSpiky(const float CurrentDistance, const float InfluenceRadius)
{
	if (FMath::IsNearlyZero(InfluenceRadius))
	{
		UE_LOG(LogTemp, Error, TEXT("GetDSpiky : InfluenceRadius is zero"));
		return 0;
	}
	auto ToDivide = PI*(FMath::Pow(InfluenceRadius,6));
	auto ToMultiply = FMath::Pow((InfluenceRadius - CurrentDistance),2);

	return -1 * (45/ToDivide) * ToMultiply;
}

/*
Poly6_2D:
W(r,h) = 4 / (π h^8) * (h^2 - r^2)^3

Spiky_2D:
W(r,h) = 10 / (π h^5) * (h - r)^3

DSpiky_2D:
dW/dr = -30 / (π h^5) * (h - r)^2
*/

float AFluidContainer::GetPoly2DKernel(const float CurrentDistance, const float InfluenceRadius)
{
	if (FMath::IsNearlyZero(InfluenceRadius))
	{
		UE_LOG(LogTemp, Error, TEXT("GetPoly2DKernel : InfluenceRadius is zero"));
		return 0;
	}
	
	auto ToDivide = (PI*(FMath::Pow(InfluenceRadius,8)));
	auto ToMultiply = FMath::Pow((InfluenceRadius*InfluenceRadius - CurrentDistance*CurrentDistance),3);
	
	return 4 / ToDivide * ToMultiply;
}


float AFluidContainer::GetSpiky2DKernel(const float CurrentDistance, const float InfluenceRadius)
{
	auto ToDivide = PI*(FMath::Pow(InfluenceRadius,5));
	if (FMath::IsNearlyZero(InfluenceRadius))
	{
		UE_LOG(LogTemp, Error, TEXT("GetSpikyKernel2D : InfluenceRadius is zero"));
		return 0;
	}
	auto ToMultiply = FMath::Pow((InfluenceRadius - CurrentDistance),3);

	return 10 / ToDivide * ToMultiply;
}

float AFluidContainer::GetDSpiky2D(const float CurrentDistance, const float InfluenceRadius)
{
	if (FMath::IsNearlyZero(InfluenceRadius))
	{
		UE_LOG(LogTemp, Error, TEXT("GetDSpiky2D : InfluenceRadius is zero"));
		return 0;
	}
	auto ToDivide = PI*(FMath::Pow(InfluenceRadius,5));
	auto ToMultiply = FMath::Pow((InfluenceRadius - CurrentDistance),2);

	return -30 / ToDivide * ToMultiply;
}

float AFluidContainer::GetMass(float WantedRestDensity, float ContainerVolume, int NbrParticules, const float /*InfluenceRadius*/)
{
	if (NbrParticules <= 0 || ContainerVolume <= 0.0f || WantedRestDensity <= 0.0f)
	{
		UE_LOG(LogTemp, Error, TEXT("Invalid mass parameters"));
		return 0.0f;
	}

	return WantedRestDensity * (ContainerVolume / NbrParticules);
}

float AFluidContainer::GetDensity(const TArray<FFluidNeighborInfo>& Neighbors, float Mass)
{
	float Density = 0;
	
	for (const FFluidNeighborInfo& Neighbor : Neighbors)
	{
		Density += Mass * Neighbor.PolyKernel;
	}
	
	return Density;
}

float AFluidContainer::GetPression(float RestDensity, float WantedRestDensity)
{
	if (FMath::IsNearlyZero(WantedRestDensity))
	{
		UE_LOG(LogTemp, Error, TEXT("GetPression : WantedRestDensity is zero"));
		return 0.0f;
	}

	return FluidInfo->PressureStiffness * (FMath::Pow(RestDensity / WantedRestDensity, K_TAIT) - 1.0f);
}

FVector2D AFluidContainer::GetPressureGradient(const TArray<FFluidNeighborInfo>& Neighbors, int ParticleId, float Mass, const TArray<float>& Densities, const TArray<float>& Pressions)
{
	FVector2D PressureGradient{0,0};

	if (!Densities.IsValidIndex(ParticleId) || !Pressions.IsValidIndex(ParticleId) || FMath::IsNearlyZero(Densities[ParticleId]))
	{
		return PressureGradient;
	}

	const float Density = Densities[ParticleId];
	const float Pression = Pressions[ParticleId];
	
	for (const FFluidNeighborInfo& Neighbor : Neighbors)
	{
		const int IdOfNeighbor = Neighbor.Id;
		if (!Densities.IsValidIndex(IdOfNeighbor) || !Pressions.IsValidIndex(IdOfNeighbor) || FMath::IsNearlyZero(Densities[IdOfNeighbor]))
		{
			continue;
		}

		const float NeighborDensity = Densities[IdOfNeighbor];
		const float NeighborPression = Pressions[IdOfNeighbor];
		const FVector2D UnitDir = !FMath::IsNearlyZero(Neighbor.Distance)
			? FVector2D(Neighbor.Delta.X / Neighbor.Distance, Neighbor.Delta.Y / Neighbor.Distance)
			: FVector2D::ZeroVector;
		PressureGradient += Mass * (Pression / FMath::Square(Density) + NeighborPression / FMath::Square(NeighborDensity)) * Neighbor.DSpiky * UnitDir;
	}
	
	
	return PressureGradient;
}

FVector2D AFluidContainer::GetViscosity(const TArray<FFluidNeighborInfo>& Neighbors, FVector2D Velocity, float Mass, const TArray<float>& Densities)
{
	
	FVector2D VelocityDivergence{0,0};
	
	for (const FFluidNeighborInfo& Neighbor : Neighbors)
	{
		const int IdOfNeighbor = Neighbor.Id;
		if (!Densities.IsValidIndex(IdOfNeighbor) || FMath::IsNearlyZero(Densities[IdOfNeighbor]))
		{
			continue;
		}

		const float NeighborDensity = Densities[IdOfNeighbor];
		if (IdOfNeighbor == INDEX_NONE || FMath::IsNearlyZero(Neighbor.Distance))
		{
			continue;
		}
		VelocityDivergence += Mass / NeighborDensity * (Particles->Velocity[IdOfNeighbor] - Velocity) * Neighbor.ViscosityWeight;
	}
	
	return 2*VelocityDivergence;
	
}

FVector2D AFluidContainer::GetAcceleration(FVector2D PressureGradient, float ViscosityStrength, FVector2D Viscosity, float Density)
{
	if (FMath::IsNearlyZero(Density))
	{
		return GetExternalForces(Density);
	}

	return PressureGradient + (ViscosityStrength * (Viscosity / Density)) + GetExternalForces(Density);
}

FVector2D AFluidContainer::GetExternalForces(float Density)
{
	if (FMath::IsNearlyZero(Density) || !IsValid(FluidInfo))
	{
		return Gravity * GravityScale;
	}

	// Gravity is authored as the acceleration wanted at rest density; convert it
	// to a force-density term, then divide by the current density.
	return Gravity * GravityScale * (FluidInfo->WantedRestDensity / Density);
}

void AFluidContainer::UpdateAutoParticleScale()
{
	if (!IsValid(StaticMeshToInstance) || !IsValid(InstancedStaticMesh))
	{
		return;
	}

	InstancedStaticMesh->SetStaticMesh(StaticMeshToInstance);
	const FVector MeshSize = StaticMeshToInstance->GetBounds().BoxExtent * 2.0f;
	const float MeshDiameter = MeshSize.GetMax();
	const float TargetParticleDiameter = ParticleRadius * 2.0f * WorldScale;
	if (MeshDiameter > UE_KINDA_SMALL_NUMBER)
	{
		MeshScale = FVector((TargetParticleDiameter / MeshDiameter) * ScaleOffset);
	}
}


// Utilities

bool AFluidContainer::GetDistanceWith(float& OutResult, const int& ParticleAId, const int& ParticleBId)
{
	// check des conditions invalide
	if (ParticleAId < 0 || ParticleBId < 0 || ParticleAId >= Particles->Position.Num() || ParticleBId >= Particles->Position.Num())
	{
		return false;
	}
	
	OutResult = FVector2D::Distance(Particles->Position[ParticleAId], Particles->Position[ParticleBId]);
	return true;
}

void AFluidContainer::GetAllNeighborsIdOf(int IdToCheck, TArray<int>& OutResult)
{
	OutResult.Empty();

	TArray<FFluidNeighborInfo> Neighbors;
	GetAllNeighborsOf(IdToCheck, Neighbors);
	OutResult.Reserve(Neighbors.Num());
	for (const FFluidNeighborInfo& Neighbor : Neighbors)
	{
		OutResult.Add(Neighbor.Id);
	}
}

void AFluidContainer::GetAllNeighborsOf(int IdToCheck, TArray<FFluidNeighborInfo>& OutResult)
{
	OutResult.Empty();

	if (!HasValidSpatialGrid() || IdToCheck < 0 || IdToCheck >= Particles->Position.Num())
	{
		return;
	}

	const FVector ParticlePosition = GetParticleSimulationPosition(IdToCheck);
	const FIntVector CenterCellCoords = GetSpatialCellCoords(ParticlePosition);
	
	for (int Z = CenterCellCoords.Z - 1; Z <= CenterCellCoords.Z + 1; ++Z)
	{
		for (int Y = CenterCellCoords.Y - 1; Y <= CenterCellCoords.Y + 1; ++Y)
		{
			for (int X = CenterCellCoords.X - 1; X <= CenterCellCoords.X + 1; ++X)
			{
				const FIntVector CellCoords(X, Y, Z);
				if (!IsSpatialCellInBounds(CellCoords))
				{
					continue;
				}

				for (int NeighborId = SpatialCellHeads[GetSpatialCellIndex(CellCoords)]; NeighborId != INDEX_NONE; NeighborId = SpatialNextParticle[NeighborId])
				{
					const FVector ToNeighbor = GetParticleSimulationPosition(NeighborId) - ParticlePosition;
					const float DistanceSquared = ToNeighbor.SizeSquared();
					if (DistanceSquared <= KernelCache.InfluenceRadiusSquared)
					{
						FFluidNeighborInfo Neighbor;
						Neighbor.Id = NeighborId;
						Neighbor.Delta = ToNeighbor;
						Neighbor.Distance = FMath::Sqrt(DistanceSquared);
						Neighbor.PolyKernel = GetCachedPoly2DKernel(DistanceSquared);
						Neighbor.DSpiky = GetCachedDSpiky2DKernel(Neighbor.Distance);
						Neighbor.ViscosityWeight = -Neighbor.DSpiky * Neighbor.Distance / (DistanceSquared + KernelCache.ViscosityDenominatorOffset);
						OutResult.Add(Neighbor);
					}
				}
			}
		}
	}
	
}
