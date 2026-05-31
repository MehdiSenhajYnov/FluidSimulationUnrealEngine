#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "FluidInfo.generated.h"

UCLASS()
class FLUIDSIMULATION_API UFluidInfo : public UDataAsset
{
	GENERATED_BODY()
public:

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid")
	int NbOfParticles;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid")
	FVector2D ContainerBounds;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid")
	float InfluenceRadius;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid", meta=(Tooltip="Densité au repos voulu"))
	float WantedRestDensity = 1000;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid", meta=(Tooltip="Force de viscosité"))
	float ViscosityStrength;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid", meta=(Tooltip="facteur de viscosité"))
	float ViscosityFactor;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fluid", meta=(Tooltip="raideur de pression, désigne à quel point la pression réagit fortement quand la densité s’éloigne de la densité au repos"))
	float PressureStiffness = 200;
};
