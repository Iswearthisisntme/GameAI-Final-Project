#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "GameAI/Grid/GAGridActor.h"
#include "Curves/CurveFloat.h"
#include "GASpatialFunction.generated.h"


UENUM(BlueprintType)
enum ESpatialInput
{
	SI_None				UMETA(DisplayName = "None"),
	SI_TargetRange		UMETA(DisplayName = "Target Range"),
	SI_PathDistance		UMETA(DisplayName = "PathDistance"),
	SI_LOS				UMETA(DisplayName = "Line Of Sight")
	// Add others if you want!
};

UENUM(BlueprintType)
enum ESpatialOp
{
	SO_None				UMETA(DisplayName = "None"),
	SO_Add				UMETA(DisplayName = "Add"),			// add this layer to the accumulated buffer
	SO_Multiply			UMETA(DisplayName = "Multiply")		// multiply this layer into the accumulated buffer
	// Add others if you want!
};


// A single layer in our spatial function
// In our simple model, we keep a single buffer (a GridMap) that accumulates the values from each 
// subsequent using an ESpatialOp -- at first this is just addition or multiplication. So you can
// express spatial functions of the form 
// Output = (((((I0 + I1) * I2) + I3) * I4) * I5)

USTRUCT(BlueprintType)
struct FFunctionLayer
{
	GENERATED_USTRUCT_BODY()

	FFunctionLayer() : Input(SI_None), Op(SO_None) {}

	UPROPERTY(BlueprintReadOnly, EditAnywhere)
	TEnumAsByte<ESpatialInput> Input;

	UPROPERTY(BlueprintReadOnly, EditAnywhere)
	FRuntimeFloatCurve ResponseCurve;

	UPROPERTY(BlueprintReadOnly, EditAnywhere)
	TEnumAsByte<ESpatialOp> Op;

};


// A spatial function is a description of how to combine various inputs (line of sight, distance, path-distance, etc.) 
// in order to rank an individual location where an AI might want to stand

UCLASS(BlueprintType, Blueprintable)
class UGASpatialFunction: public UObject
{
	GENERATED_UCLASS_BODY()

	// Our list of layers
	UPROPERTY(BlueprintReadOnly, EditAnywhere)
	TArray<FFunctionLayer> Layers;
};