#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "GameAI/Grid/GAGridMap.h"
#include "GATargetComponent.generated.h"


class AGAGridActor;

UENUM(BlueprintType)
enum ETargetState
{
	GATS_Unknown		UMETA(DisplayName = "Unknown"),			// I have never been seen
	GATS_Immediate		UMETA(DisplayName = "Immediate"),		// I am currently being observed
	GATS_Hidden			UMETA(DisplayName = "Hidden"),			// I am known about but am not currently observed
};


// Cached information about a target
USTRUCT(BlueprintType)
struct FTargetCache
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(BlueprintReadOnly)
	TEnumAsByte<ETargetState> State;

	// The last known position of the target in world space
	UPROPERTY(BlueprintReadOnly)
	FVector Position;

	// The last known velocity of the target in world space
	UPROPERTY(BlueprintReadOnly)
	FVector Velocity;

	void Set(const FVector& NewPosition, const FVector& NewVelocity)
	{
		Position = NewPosition;
		Velocity = NewVelocity;
	}

};


UCLASS(BlueprintType, Blueprintable, meta = (BlueprintSpawnableComponent))
class UGATargetComponent : public UActorComponent
{
	GENERATED_UCLASS_BODY()

	// A unique GUID generated for each target at constructor time
	// Used as a unique ID for targets
	UPROPERTY(BlueprintReadOnly)
	FGuid TargetGuid;

	// Last known state of the target
	UPROPERTY(BlueprintReadOnly)
	FTargetCache LastKnownState;
	
	// Occupancy Map

	UPROPERTY(BlueprintReadOnly)
	FGAGridMap OccupancyMap;

	UPROPERTY(BlueprintReadOnly)
	bool bDebugOccupancyMap = true;


	// Cached pointer to the grid actor
	UPROPERTY()
	mutable TSoftObjectPtr<AGAGridActor> GridActor;

	// ACCESSORS ----------

	// Get last known information about this target
	UFUNCTION(BlueprintCallable)
	FTargetCache GetTargetCache() const
	{
		return LastKnownState;
	}

	UFUNCTION(BlueprintCallable)
	AGAGridActor *GetGridActor() const;

	// Return TRUE if at least ONE AI has reach Awareness == 1 for this target
	bool IsKnown() const
	{
		return (LastKnownState.State == GATS_Immediate) || (LastKnownState.State == GATS_Hidden);
	}


	// UPDATE ----------

	// Needed for some bookkeeping (registering the target component with the the Perception System)
	// You shouldn't need to touch these.
	virtual void OnRegister() override;
	virtual void OnUnregister() override;

	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	void OccupancyMapSetPosition(const FVector &Position);
	void OccupancyMapUpdate();
	void OccupancyMapDiffuse();

};