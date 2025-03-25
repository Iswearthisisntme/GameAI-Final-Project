#include "GAPerceptionComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GAPerceptionSystem.h"

UGAPerceptionComponent::UGAPerceptionComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;

	// Default vision parameters
	VisionParameters.VisionAngle = 90.0f;
	VisionParameters.VisionDistance = 2500.0;
}


void UGAPerceptionComponent::OnRegister()
{
	Super::OnRegister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->RegisterPerceptionComponent(this);
	}
}

void UGAPerceptionComponent::OnUnregister()
{
	Super::OnUnregister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->UnregisterPerceptionComponent(this);
	}
}


APawn* UGAPerceptionComponent::GetOwnerPawn() const
{
	AActor* Owner = GetOwner();
	if (Owner)
	{
		APawn* Pawn = Cast<APawn>(Owner);
		if (Pawn)
		{
			return Pawn;
		}
		else
		{
			AController* Controller = Cast<AController>(Owner);
			if (Controller)
			{
				return Controller->GetPawn();
			}
		}
	}

	return NULL;
}



// Returns the Target this AI is attending to right now.

UGATargetComponent* UGAPerceptionComponent::GetCurrentTarget() const
{
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);

	if (PerceptionSystem && PerceptionSystem->TargetComponents.Num() > 0)
	{
		UGATargetComponent* TargetComponent = PerceptionSystem->TargetComponents[0];
		if (TargetComponent->IsKnown())
		{
			return PerceptionSystem->TargetComponents[0];
		}
	}

	return NULL;
}

bool UGAPerceptionComponent::HasTarget() const
{
	return GetCurrentTarget() != NULL;
}


bool UGAPerceptionComponent::GetCurrentTargetState(FTargetCache& TargetStateOut, FTargetData& TargetDataOut) const
{
	UGATargetComponent* Target = GetCurrentTarget();
	if (Target)
	{
		const FTargetData* TargetData = TargetMap.Find(Target->TargetGuid);
		if (TargetData)
		{
			TargetStateOut = Target->LastKnownState;
			TargetDataOut = *TargetData;
			return true;
		}

	}
	return false;
}


void UGAPerceptionComponent::GetAllTargetStates(bool OnlyKnown, TArray<FTargetCache>& TargetCachesOut, TArray<FTargetData>& TargetDatasOut) const
{
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGATargetComponent>>& TargetComponents = PerceptionSystem->GetAllTargetComponents();
		for (UGATargetComponent* TargetComponent : TargetComponents)
		{
			const FTargetData* TargetData = TargetMap.Find(TargetComponent->TargetGuid);
			if (TargetData)
			{
				if (!OnlyKnown || TargetComponent->IsKnown())
				{
					TargetCachesOut.Add(TargetComponent->LastKnownState);
					TargetDatasOut.Add(*TargetData);
				}
			}
		}
	}
}


void UGAPerceptionComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Update all target data as usual.
	UpdateAllTargetData();

	// Get the AI pawn that owns this component.
	APawn* OwnerPawn = GetOwnerPawn();
	if (!OwnerPawn)
	{
		return;
	}

	// Retrieve the last known target state.
	FTargetCache LastKnownTarget;
	FTargetData DummyTargetData;
	if (GetCurrentTargetState(LastKnownTarget, DummyTargetData))
	{
		// Compute the direction from the pawn to the target's last known position.
		FVector ToTarget = LastKnownTarget.Position - OwnerPawn->GetActorLocation();
		if (!ToTarget.IsNearlyZero())
		{
			// Get the desired rotation to face the target.
			FRotator DesiredRotation = ToTarget.Rotation();

			// Optionally, you can interpolate smoothly for a gradual turn:
			FRotator CurrentRotation = OwnerPawn->GetActorRotation();
			FRotator NewRotation = FMath::RInterpTo(CurrentRotation, DesiredRotation, DeltaTime, 5.0f);
			NewRotation.Roll = 0.0f;
			NewRotation.Pitch = 0.0f;
			OwnerPawn->SetActorRotation(NewRotation);

			// For an immediate rotation update, set it directly:
			OwnerPawn->SetActorRotation(NewRotation);
		}
	}
}


void UGAPerceptionComponent::UpdateAllTargetData()
{
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGATargetComponent>>& TargetComponents = PerceptionSystem->GetAllTargetComponents();
		for (UGATargetComponent* TargetComponent : TargetComponents)
		{
			UpdateTargetData(TargetComponent);
		}
	}
}

void UGAPerceptionComponent::UpdateTargetData(UGATargetComponent* TargetComponent)
{
	// REMEMBER: the UGAPerceptionComponent is going to be attached to the controller, not the pawn. So we call this special accessor to 
	// get the pawn that our controller is controlling
	APawn* OwnerPawn = GetOwnerPawn();
	if (!OwnerPawn || !TargetComponent)
	{
		return;
	}

	// Retrieve target actor
	AActor* TargetActor = TargetComponent->GetOwner();
	if (!TargetActor)
	{
		return;
	}
	FTargetData* TargetData = TargetMap.Find(TargetComponent->TargetGuid);
	if (TargetData == NULL)		// If we don't already have a target data for the given target component, add it
	{
		FTargetData NewTargetData;
		FGuid TargetGuid = TargetComponent->TargetGuid;
		TargetData = &TargetMap.Add(TargetGuid, NewTargetData);
	}


	// TODO PART 3
	// 
	// - Update TargetData->bClearLOS
	//		Use this.VisionParameters to determine whether the target is within the vision cone or not 
	//		(and ideally do so before you cast a ray towards it)
	// - Update TargetData->Awareness
	//		On ticks when the AI has a clear LOS, the Awareness should grow
	//		On ticks when the AI does not have a clear LOS, the Awareness should decay
	//
	// Awareness should be clamped to the range [0, 1]
	// You can add parameters to the UGAPerceptionComponent to control the speed at which awareness rises and falls

	// YOUR CODE HERE

	FVector AIPosition = OwnerPawn->GetActorLocation();
	FVector TargetPosition = TargetActor->GetActorLocation();
	FVector DirectionToTarget = (TargetPosition - AIPosition).GetSafeNormal();
	FVector AIForwardVector = OwnerPawn->GetActorForwardVector();
	float DotProduct = FVector::DotProduct(DirectionToTarget, AIForwardVector);
	float VisionThreshold = FMath::Cos(FMath::DegreesToRadians(VisionParameters.VisionAngle * 0.5f));

	bool bWithinVisionCone = (DotProduct >= VisionThreshold);
	bool bWithinVisionRange = (FVector::Dist(AIPosition, TargetPosition) <= VisionParameters.VisionDistance);
	bool bHasClearLOS = false;

	if (bWithinVisionCone && bWithinVisionRange)
	{
		// Perform Line Trace for LOS check
		FHitResult Hit;
		FCollisionQueryParams TraceParams;
		TraceParams.AddIgnoredActor(OwnerPawn);
		TraceParams.AddIgnoredActor(TargetActor);

		bool bBlocked = GetWorld()->LineTraceSingleByChannel(
			Hit, AIPosition, TargetPosition, ECC_Visibility, TraceParams);

		bHasClearLOS = !bBlocked;
	}

	// Update LOS status
	TargetData->bClearLos = bHasClearLOS;

	// Awareness Handling
	const float AwarenessGain = 0.7f;
	const float AwarenessLoss = 0.2f;

	float DeltaTime = GetWorld()->GetDeltaSeconds();

	if (bHasClearLOS)
	{
		// Increase awareness
		TargetData->Awareness = FMath::Clamp(TargetData->Awareness + AwarenessGain * DeltaTime, 0.0f, 1.0f);
	}
	else
	{
		// Decrease awareness
		TargetData->Awareness = FMath::Clamp(TargetData->Awareness - AwarenessLoss * DeltaTime, 0.0f, 1.0f);
	}
}



const FTargetData* UGAPerceptionComponent::GetTargetData(FGuid TargetGuid) const
{
	return TargetMap.Find(TargetGuid);
}