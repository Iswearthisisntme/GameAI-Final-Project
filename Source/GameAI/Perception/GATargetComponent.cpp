#include "GATargetComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GameAI/Grid/GAGridActor.h"
#include "GAPerceptionSystem.h"
#include "ProceduralMeshComponent.h"
#include "GameAI/Perception/GAPerceptionComponent.h"



UGATargetComponent::UGATargetComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;

	SetTickGroup(ETickingGroup::TG_PostUpdateWork);

	// Generate a new guid
	TargetGuid = FGuid::NewGuid();
}


AGAGridActor* UGATargetComponent::GetGridActor() const
{
	AGAGridActor* Result = GridActor.Get();
	if (Result)
	{
		return Result;
	}
	else
	{
		AActor* GenericResult = UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass());
		if (GenericResult)
		{
			Result = Cast<AGAGridActor>(GenericResult);
			if (Result)
			{
				// Cache the result
				// Note, GridActor is marked as mutable in the header, which is why this is allowed in a const method
				GridActor = Result;
			}
		}

		return Result;
	}
}


void UGATargetComponent::OnRegister()
{
	Super::OnRegister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->RegisterTargetComponent(this);
	}

	const AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		OccupancyMap = FGAGridMap(Grid, 0.0f);
	}
}

void UGATargetComponent::OnUnregister()
{
	Super::OnUnregister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->UnregisterTargetComponent(this);
	}
}



void UGATargetComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	bool isImmediate = false;

	// update my perception state FSM
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGAPerceptionComponent>>& PerceptionComponents = PerceptionSystem->GetAllPerceptionComponents();
		for (UGAPerceptionComponent* PerceptionComponent : PerceptionComponents)
		{
			const FTargetData* TargetData = PerceptionComponent->GetTargetData(TargetGuid);
			if (TargetData && (TargetData->Awareness >= 1.0f))
			{
				isImmediate = true;
				break;
			}
		}
	}

	if (isImmediate)
	{
		AActor* Owner = GetOwner();
		LastKnownState.State = GATS_Immediate;

		// REFRESH MY STATE
		LastKnownState.Set(Owner->GetActorLocation(), Owner->GetVelocity());

		// Tell the omap to clear out and put all the probability in the observed location
		OccupancyMapSetPosition(LastKnownState.Position);
	}
	else if (IsKnown())
	{
		LastKnownState.State = GATS_Hidden;
	}

	if (LastKnownState.State == GATS_Hidden)
	{
		OccupancyMapUpdate();
	}

	// As long as I'm known, whether I'm immediate or not, diffuse the probability in the omap

	if (IsKnown())
	{
		OccupancyMapDiffuse();
	}

	if (bDebugOccupancyMap)
	{
		AGAGridActor* Grid = GetGridActor();
		Grid->DebugGridMap = OccupancyMap;
		GridActor->RefreshDebugTexture();
		GridActor->DebugMeshComponent->SetVisibility(true);
	}
}

void UGATargetComponent::OccupancyMapSetPosition(const FVector& Position)
{
	// TODO PART 4

	// We've been observed to be in a given position
	// Clear out all probability in the omap, and set the appropriate cell to P = 1.0
	if (AGAGridActor* Grid = GetGridActor())
	{
		OccupancyMap.ResetData(0.0f);
		if (FCellRef Cell = Grid->GetCellRef(Position, true); Cell.IsValid())
		{
			OccupancyMap.SetValue(Cell, 1.0f);
		}
	}
}


void UGATargetComponent::OccupancyMapUpdate()
{
	const AGAGridActor* Grid = GetGridActor();
	if (!Grid) return;

	FGAGridMap VisibilityGrid(Grid, 0.0f);

	// TODO PART 4

	// STEP 1: Build a visibility map, based on the perception components of the AIs in the world
	// The visibility map is a simple map where each cell is either 0 (not currently visible to ANY perceiver) or 1 (currently visible to one or more perceivers).
	// 
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		const TArray<TObjectPtr<UGAPerceptionComponent>>& PerceptionComponents = PerceptionSystem->GetAllPerceptionComponents();

		for (UGAPerceptionComponent* PerceptionComp : PerceptionComponents)
		{
			// Find visible cells for this perceiver.
			// Reminder: Use the PerceptionComponent.VisionParameters when determining whether a cell is visible or not (in addition to a line trace).
			// Suggestion: you might find it useful to add a UGAPerceptionComponent::TestVisibility method to the perception component.
			APawn* AIActor = PerceptionComp->GetOwnerPawn();
			if (!AIActor) continue;

			FVector AIPosition = AIActor->GetActorLocation();
			FVector ForwardVector = AIActor->GetActorForwardVector();
			float VisionRadius = PerceptionComp->VisionParameters.VisionDistance;
			float VisionAngleCos = FMath::Cos(FMath::DegreesToRadians(PerceptionComp->VisionParameters.VisionAngle * 0.5f));

			for (int32 X = 0; X < Grid->XCount; X++)
			{
				for (int32 Y = 0; Y < Grid->YCount; Y++)
				{
					FCellRef Cell(X, Y);
					FVector CellPosition = Grid->GetCellPosition(Cell);
					FVector DirectionToCell = (CellPosition - AIPosition).GetSafeNormal();

					bool bInCone = FVector::DotProduct(DirectionToCell, ForwardVector) >= VisionAngleCos;
					bool bInRange = FVector::Dist(AIPosition, CellPosition) <= VisionRadius;
					bool bHasLineOfSight = false;

					if (bInCone && bInRange)
					{
						FHitResult Hit;
						FCollisionQueryParams QueryParams;
						QueryParams.AddIgnoredActor(AIActor);

						bHasLineOfSight = !AIActor->GetWorld()->LineTraceSingleByChannel(
							Hit, AIPosition, CellPosition, ECC_Visibility, QueryParams);
					}

					if (bHasLineOfSight)
					{
						VisibilityGrid.SetValue(Cell, 1.0f);
					}
				}
			}
		}
	}

	// STEP 2: Clear out the probability in the visible cells
	for (int32 X = 0; X < Grid->XCount; X++)
	{
		for (int32 Y = 0; Y < Grid->YCount; Y++)
		{
			FCellRef Cell(X, Y);
			float Visibility = 0.0f;
			if (VisibilityGrid.GetValue(Cell, Visibility) && Visibility > 0.0f)
			{
				OccupancyMap.SetValue(Cell, 0.0f);
			}
		}
	}

	// STEP 3: Renormalize the OMap, so that it's still a valid probability distribution
	float TotalProb = 0.0f;
	for (int32 X = 0; X < Grid->XCount; X++)
	{
		for (int32 Y = 0; Y < Grid->YCount; Y++)
		{
			FCellRef Cell(X, Y);
			float Prob = 0.0f;
			if (OccupancyMap.GetValue(Cell, Prob))
			{
				TotalProb += Prob;
			}
		}
	}

	if (TotalProb > 0.0f)
	{
		for (int32 X = 0; X < Grid->XCount; X++)
		{
			for (int32 Y = 0; Y < Grid->YCount; Y++)
			{
				FCellRef Cell(X, Y);
				float Prob = 0.0f;
				if (OccupancyMap.GetValue(Cell, Prob))
				{
					OccupancyMap.SetValue(Cell, Prob / TotalProb);
				}
			}
		}
	}

	// Ensure non-traversable cells have zero probability
	for (int32 X = 0; X < Grid->XCount; X++)
	{
		for (int32 Y = 0; Y < Grid->YCount; Y++)
		{
			FCellRef Cell(X, Y);
			if (!(Grid->GetCellData(Cell) & ECellData::CellDataTraversable))
			{
				OccupancyMap.SetValue(Cell, 0.0f);
			}
		}
	}

	float NewTotalProb = 0.0f;
	for (int32 X = 0; X < Grid->XCount; X++)
	{
		for (int32 Y = 0; Y < Grid->YCount; Y++)
		{
			FCellRef Cell(X, Y);
			float Prob = 0.0f;
			if (OccupancyMap.GetValue(Cell, Prob))
			{
				NewTotalProb += Prob;
			}
		}
	}

	if (NewTotalProb > 0.0f)
	{
		for (int32 X = 0; X < Grid->XCount; X++)
		{
			for (int32 Y = 0; Y < Grid->YCount; Y++)
			{
				FCellRef Cell(X, Y);
				float Prob = 0.0f;
				if (OccupancyMap.GetValue(Cell, Prob))
				{
					OccupancyMap.SetValue(Cell, Prob / NewTotalProb);
				}
			}
		}
	}

	// STEP 4: Extract the highest-likelihood cell on the omap and refresh the LastKnownState.
	FVector WeightedSum = FVector::ZeroVector;
	float SumProbability = 0.0f;

	for (int32 X = 0; X < Grid->XCount; X++)
	{
		for (int32 Y = 0; Y < Grid->YCount; Y++)
		{
			FCellRef Cell(X, Y);
			float Prob = 0.0f;
			if (OccupancyMap.GetValue(Cell, Prob))
			{
				WeightedSum += Grid->GetCellPosition(Cell) * Prob;
				SumProbability += Prob;
			}
		}
	}

	if (SumProbability > 0.0f)
	{
		LastKnownState.Set(WeightedSum / SumProbability, FVector::ZeroVector);
	}
}



void UGATargetComponent::OccupancyMapDiffuse()
{
	// TODO PART 4
	// Diffuse the probability in the OMAP
	if (const AGAGridActor* Grid = GetGridActor())
	{
		const float DiffusionRate = 0.1f;
		FGAGridMap NewMap(Grid, 0.0f);

		for (int32 X = 0; X < Grid->XCount; X++)
		{
			for (int32 Y = 0; Y < Grid->YCount; Y++)
			{
				FCellRef Cell(X, Y);
				float OldValue = 0.0f;
				OccupancyMap.GetValue(Cell, OldValue);

				float NeighborSum = 0.0f;
				TArray<FCellRef> Neighbors = { FCellRef(X + 1, Y), FCellRef(X - 1, Y), FCellRef(X, Y + 1), FCellRef(X, Y - 1) };

				for (const FCellRef& Neighbor : Neighbors)
				{
					if (Grid->IsCellRefInBounds(Neighbor))
					{
						float NeighborValue = 0.0f;
						OccupancyMap.GetValue(Neighbor, NeighborValue);
						NeighborSum += NeighborValue;
					}
				}

				NewMap.SetValue(Cell, (1.0f - DiffusionRate) * OldValue + DiffusionRate * NeighborSum);
			}
		}

		OccupancyMap = NewMap;
	}
}