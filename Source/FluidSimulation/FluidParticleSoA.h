// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "FluidParticleSoA.generated.h"

/**
 * 
 */
UCLASS()
class FLUIDSIMULATION_API UFluidParticleSoA : public UObject
{
	GENERATED_BODY()
public:
	void Init(int NbOfParticles, FVector2D StartPosition, FVector2D ContainerBounds, float SpaceBetween)
	{
		if (NbOfParticles <= 0 || SpaceBetween <= 0.0f || ContainerBounds.X <= 0.0f || ContainerBounds.Y <= 0.0f)
		{
			UE_LOG(LogTemp, Error, TEXT("Invalid fluid particle init parameters"));
			Id.Reset();
			Position.Reset();
			Velocity.Reset();
			Acceleration.Reset();
			AccelerationToUse.Reset();
			return;
		}
		
		const int MaxColumns = FMath::Max(1, FMath::FloorToInt(ContainerBounds.X / SpaceBetween) + 1);
		const int MaxRows = FMath::Max(1, FMath::FloorToInt(ContainerBounds.Y / SpaceBetween) + 1);
		const int MaxParticlesInBounds = MaxColumns * MaxRows;
		const int ParticlesToSpawn = FMath::Min(NbOfParticles, MaxParticlesInBounds);
		
		if (ParticlesToSpawn < NbOfParticles)
		{
			UE_LOG(LogTemp, Warning, TEXT("Fluid particle count clamped from %d to %d to fit container bounds"), NbOfParticles, ParticlesToSpawn);
		}
		
		Id.SetNum(ParticlesToSpawn);
		Position.SetNum(ParticlesToSpawn);
		Velocity.SetNum(ParticlesToSpawn);
		Acceleration.SetNum(ParticlesToSpawn);
		AccelerationToUse.SetNum(ParticlesToSpawn);
		
		FVector2D PositionToUse = StartPosition;
		
		for (int i = 0; i < ParticlesToSpawn; i++)
		{
			Id[i] = i;
			Position[i] = PositionToUse;
			Velocity[i] = FVector2D::ZeroVector;
			Acceleration[i] = FVector2D::ZeroVector;
			AccelerationToUse[i] = FVector2D::ZeroVector;
			
			PositionToUse += FVector2D(SpaceBetween,0);
			if (PositionToUse.X > StartPosition.X + ContainerBounds.X + UE_KINDA_SMALL_NUMBER)
			{
				PositionToUse.X = StartPosition.X;
				PositionToUse.Y += SpaceBetween;
			}
		}
	}
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid")
	TArray<int> Id;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid")
	TArray<FVector2D> Position;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid")
	TArray<FVector2D> Velocity;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid")
	TArray<FVector2D> Acceleration;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid")
	TArray<FVector2D> AccelerationToUse;
	
};
