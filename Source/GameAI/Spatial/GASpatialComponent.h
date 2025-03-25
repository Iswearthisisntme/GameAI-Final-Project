#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "GameAI/Grid/GAGridActor.h"
#include "GASpatialComponent.generated.h"

class UGASpatialFunction;
struct FFunctionLayer;
class AGAGridActor;
class UGAPathComponent;

// Our spatial component
// This component is going to help make us make decisions about where to stand
// Note: this should go on the AI's controller, not the pawn.

UCLASS(BlueprintType, Blueprintable, meta = (BlueprintSpawnableComponent))
class UGASpatialComponent : public UActorComponent
{
	GENERATED_UCLASS_BODY()

	// This is the spatial function we will use to 
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	TSubclassOf<UGASpatialFunction> SpatialFunctionReference;

	// Choose a position from within a box of this size (on each side) centered on the owner
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float SampleDimensions;

	// A couple of cached pointers and associated accessors for convenience

	UPROPERTY()
	mutable TSoftObjectPtr<AGAGridActor> GridActor;

	UPROPERTY()
	mutable TSoftObjectPtr<UGAPathComponent> PathComponent;

	UFUNCTION(BlueprintCallable)
	const AGAGridActor *GetGridActor() const;

	UFUNCTION(BlueprintCallable)
	const UGAPathComponent *GetPathComponent() const;


	// It is super easy to forget: this component will usually be attached to the CONTROLLER, not the pawn it's controlling
	// A lot of times we want access to the pawn (e.g. when sending signals to its movement component).
	UFUNCTION(BlueprintCallable, BlueprintPure)
	APawn *GetOwnerPawn() const;


	// Core functionality

	UFUNCTION(BlueprintCallable)
	bool ChoosePosition(bool PathfindToPosition, bool Debug);

	void EvaluateLayer(const FFunctionLayer& Layer, FGAGridMap& GridMap, const FGAGridMap& DistanceMap) const;

	protected:
		// Fix: Declare OccupancyMap here
		FGAGridMap OccupancyMap;

};