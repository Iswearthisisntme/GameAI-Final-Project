#include "GASpatialComponent.h"
#include "GameAI/Pathfinding/GAPathComponent.h"
#include "GameAI/Grid/GAGridMap.h"
#include "Kismet/GameplayStatics.h"
#include "Math/MathFwd.h"
#include "GASpatialFunction.h"
#include "ProceduralMeshComponent.h"
#include "GameAI/Perception/GAPerceptionComponent.h" 



UGASpatialComponent::UGASpatialComponent(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    SampleDimensions = 8000.0f;		// should cover the bulk of the test map
}


const AGAGridActor* UGASpatialComponent::GetGridActor() const
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

const UGAPathComponent* UGASpatialComponent::GetPathComponent() const
{
    UGAPathComponent* Result = PathComponent.Get();
    if (Result)
    {
        return Result;
    }
    else
    {
        AActor* Owner = GetOwner();
        if (Owner)
        {
            // Note, the UGAPathComponent and the UGASpatialComponent are both on the controller
            Result = Owner->GetComponentByClass<UGAPathComponent>();
            if (Result)
            {
                PathComponent = Result;
            }
        }
        return Result;
    }
}

APawn* UGASpatialComponent::GetOwnerPawn() const
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

bool UGASpatialComponent::ChoosePosition(bool PathfindToPosition, bool Debug)
{
    bool Result = false;
    const APawn* OwnerPawn = GetOwnerPawn();
    const AGAGridActor* Grid = GetGridActor();

    if (SpatialFunctionReference.Get() == NULL)
    {
        UE_LOG(LogTemp, Warning, TEXT("UGASpatialComponent has no SpatialFunctionReference assigned."));
        return false;
    }

    if (GridActor == NULL)
    {
        UE_LOG(LogTemp, Warning, TEXT("UGASpatialComponent::ChoosePosition can't find a GridActor."));
        return false;
    }

    // Don't worry too much about the Unreal-ism below. Technically our SpatialFunctionReference is not ACTUALLY
    // a spatial function instance, rather it's a class, which happens to have a lot of data in it.
    // Happily, Unreal creates, under the hood, a default object for every class, that lets you access that data
    // as if it were a normal instance
    const UGASpatialFunction* SpatialFunction = SpatialFunctionReference->GetDefaultObject<UGASpatialFunction>();

    // The below is to create a GridMap (which you will fill in) based on a bounding box centered around the OwnerPawn

    FBox2D Box(EForceInit::ForceInit);
    FIntRect CellRect;
    FVector2D PawnLocation(OwnerPawn->GetActorLocation());
    Box += PawnLocation;
    Box = Box.ExpandBy(SampleDimensions / 2.0f);
    if (GridActor->GridSpaceBoundsToRect2D(Box, CellRect))
    {
        // Super annoying, by the way, that FIntRect is not blueprint accessible, because it forces us instead
        // to make a separate bp-accessible FStruct that represents _exactly the same thing_.
        FGridBox GridBox(CellRect);

        // This is the grid map I'm going to fill with values
        FGAGridMap GridMap(Grid, GridBox, 0.0f);

        // Fill in this distance map using Dijkstra!
        FGAGridMap DistanceMap(Grid, GridBox, FLT_MAX);

        // ~~~ STEPS TO FILL IN FOR ASSIGNMENT 3 ~~~

        // Step 1: Run Dijkstra's to determine which cells we should even be evaluating (the GATHER phase)
        // (You should add a Dijkstra() function to the UGAPathComponent())
        // I would recommend adding a method to the path component which looks something like
        // bool UGAPathComponent::Dijkstra(const FVector &StartPoint, FGAGridMap &DistanceMapOut) const;

        //getting the path component
        GetPathComponent();
        if (!PathComponent)
        {
            UE_LOG(LogTemp, Warning, TEXT("PathComponent is not found! "));
            return false;
        }

        FVector StartPoint = OwnerPawn->GetActorLocation();
        if (!PathComponent->Dijkstra(StartPoint, DistanceMap))
        {
            UE_LOG(LogTemp, Warning, TEXT("Dijkstra has failed!!!"));
            return false;
        }

        // Step 2: For each layer in the spatial function, evaluate and accumulate the layer in GridMap
        // Note, only evaluate accessible cells found in step 1
        for (const FFunctionLayer& Layer : SpatialFunction->Layers)
        {
            EvaluateLayer(Layer, GridMap, DistanceMap);
        }

        // Step 3: pick the best cell in GridMap

        //// Let's pretend for now we succeeded.
        //Result = true;

        FCellRef BestCell;
        float BestValue = FLT_MAX;

        for (int32 Y = GridMap.GridBounds.MinY; Y < GridMap.GridBounds.MaxY; Y++)
        {
            for (int32 X = GridMap.GridBounds.MinX; X < GridMap.GridBounds.MaxX; X++)
            {
                FCellRef CellRef(X, Y);

                if (EnumHasAllFlags(Grid->GetCellData(CellRef), ECellData::CellDataTraversable))
                {
                    float CellDistance = FLT_MAX;

                    if (DistanceMap.GetValue(CellRef, CellDistance) && CellDistance < FLT_MAX)
                    {
                        float CellValue = 0.0f;
                        GridMap.GetValue(CellRef, CellValue);
                        if (CellValue < BestValue)
                        {
                            BestValue = CellValue;
                            BestCell = CellRef;
                        }
                    }
                }
            }
        }

        if (!BestCell.IsValid())
        {
            UE_LOG(LogTemp, Warning, TEXT("No bestcell found! "));
            return false;
        }

        if (PathfindToPosition)
        {
            // Step 4: Go there!
            // This will involve reconstructing the path and then getting it into the UGAPathComponent
            // Depending on what your cached Dijkstra data looks like, the path reconstruction might be implemented here
            // or in the UGAPathComponent

            FCellRef StartCell = Grid->GetCellRef(StartPoint);
            TArray<FPathStep> UnsmoothedPath;
            if (PathComponent->ReconstructPath(DistanceMap, BestCell, StartCell, UnsmoothedPath))
            {
                TArray<FPathStep> SmoothedPath;
                if (PathComponent->SmoothPath(StartPoint, UnsmoothedPath, SmoothedPath) == GAPS_Active)
                {
                    PathComponent->Steps = SmoothedPath;
                    FVector BestPos = Grid->GetCellPosition(BestCell);
                    PathComponent->SetDestination(BestPos);
                }
                else
                {
                    UE_LOG(LogTemp, Warning, TEXT("Smooth path failed"));
                }
            }
            else
            {
                UE_LOG(LogTemp, Warning, TEXT("reconstructpath faild"));
            }
        }

        Result = true;

        if (Debug)
        {
            // Note: this outputs (basically) the results of the position selection
            // However, you can get creative with the debugging here. For example, maybe you want
            // to be able to examine the values of a specific layer in the spatial function
            // You could create a separate debug map above (where you're doing the evaluations) and
            // cache it off for debug rendering. Ideally you'd be able to control what layer you wanted to 
            // see from blueprint

            GridActor->DebugGridMap = GridMap;
            GridActor->RefreshDebugTexture();
            GridActor->DebugMeshComponent->SetVisibility(true);
        }
    }

    return Result;
}


void UGASpatialComponent::EvaluateLayer(const FFunctionLayer& Layer, FGAGridMap& GridMap, const FGAGridMap& DistanceMap) const
{
    // Retrieve the perception component from our owner.
    UGAPerceptionComponent* PerceptionComponent = GetOwner()->FindComponentByClass<UGAPerceptionComponent>();
    FTargetCache TargetCache;
    FTargetData TargetData;
    bool bHasTarget = PerceptionComponent && PerceptionComponent->GetCurrentTargetState(TargetCache, TargetData);

    const AGAGridActor* Grid = GetGridActor();
    if (!Grid)
    {
        UE_LOG(LogTemp, Warning, TEXT("UGASpatialComponent::EvaluateLayer: Grid Actor is null."));
        return;
    }
    // Loop over every cell in the grid map.
    for (int32 Y = GridMap.GridBounds.MinY; Y < GridMap.GridBounds.MaxY; Y++)
    {
        for (int32 X = GridMap.GridBounds.MinX; X < GridMap.GridBounds.MaxX; X++)
        {

            FCellRef CellRef(X, Y);
            
            // Make sure it's traversable. NO MORE TRYING TO GO OUTSIDE OF THE WORLD. thanks discord
            if (EnumHasAllFlags(Grid->GetCellData(CellRef), ECellData::CellDataTraversable))
            {

                float InputValue = 0.0f;

                // evaluate me!

                // First step is determine input value. Remember there are three possible inputs to handle:
                // 	SI_None				UMETA(DisplayName = "None"),
                //	SI_TargetRange		UMETA(DisplayName = "Target Range"),
                //	SI_PathDistance		UMETA(DisplayName = "PathDistance"),
                //	SI_LOS				UMETA(DisplayName = "Line Of Sight")


                FVector CellPos = Grid->GetCellPosition(CellRef);

                // Choose how to evaluate this layer based on its type.
                switch (Layer.Input)
                {
                case SI_TargetRange:
                {
                    if (bHasTarget)
                    {
                        // Use the last known target position.
                        float ActualDistance = FVector::Dist(CellPos, TargetCache.Position);
                        float DesiredDistance = Layer.Input;
                        InputValue = FMath::Abs(ActualDistance - DesiredDistance);
                    }
                    else
                    {
                        InputValue = FLT_MAX;
                    }
                    break;
                }
                case SI_PathDistance:
                {
                    float DistValue = 0.0f;
                    if (DistanceMap.GetValue(CellRef, DistValue) && DistValue < FLT_MAX)
                    {
                        InputValue = DistValue;
                    }
                    else
                    {
                        InputValue = FLT_MAX; // Unreachable cells
                    }
                    break;
                }
                case SI_LOS:
                {
                    if (bHasTarget)
                    {

                        UWorld* World = GetWorld();
                        if (!World)
                        {
                            UE_LOG(LogTemp, Warning, TEXT("UGASpatialComponent::EvaluateLayer: World is null."));
                            return;
                        }

                        FHitResult HitResult;
                        FCollisionQueryParams Params;
                        APawn* OwnerPawn = GetOwnerPawn();
                        if (OwnerPawn)
                        {
                            Params.AddIgnoredActor(OwnerPawn);
                        }
                        // Line-of-sight check using the last known target position.
                        FVector TargetLoc = TargetCache.Position;
                        CellPos.Z = TargetLoc.Z;
                        bool bHit = World->LineTraceSingleByChannel(HitResult, CellPos, TargetLoc, ECC_Visibility, Params);
                        // If hit, then LOS is blocked.
                        InputValue = bHit ? 0.0f : 1.0f;
                    }
                    else
                    {
                        InputValue = 1.0f; // Assume LOS blocked when target is unknown.
                    }
                    break;
                }
                case SI_None:
                default:
                    InputValue = 0.0f;
                    break;
                }

                // Responsive curve part
                // Next, run it through the response curve using something like this
                // float Value = 4.5f;
                // float ModifiedValue = Layer.ResponseCurve.GetRichCurveConst()->Eval(Value, 0.0f);

                // Then add it's influence to the grid map, combining with the current value using one of the two operators
                //	SO_None				UMETA(DisplayName = "None"),
                //	SO_Add				UMETA(DisplayName = "Add"),			// add this layer to the accumulated buffer
                //	SO_Multiply			UMETA(DisplayName = "Multiply")		// multiply this layer into the accumulated buffer

                //GridMap.SetValue(CellRef, CombinedValue);

                float CurveValue = 0.0f;
                if (Layer.ResponseCurve.GetRichCurveConst())
                {
                    CurveValue = Layer.ResponseCurve.GetRichCurveConst()->Eval(InputValue, 0.0f);
                }
                else
                {
                    CurveValue = InputValue;
                }

                // Get the current accumulated value from the grid map.
                float CurrentValue = 0.0f;
                GridMap.GetValue(CellRef, CurrentValue);

                // Instead of overwriting when operator is SO_None, accumulate additively.
                float CombinedValue = 0.0f;
                switch (Layer.Op)
                {
                case SO_None:
                    break;

                case SO_Add:
                    CombinedValue = CurrentValue + CurveValue;
                    break;

                case SO_Multiply:
                    CombinedValue = CurrentValue * CurveValue;
                    break;

                default:
                    break;
                }

                GridMap.SetValue(CellRef, CombinedValue);

            }
            else {
                continue;
            }
        }
    }
    // HERE ARE SOME ADDITIONAL HINTS

    // Here's how to get the player's pawn
    // APawn *PlayerPawn = UGameplayStatics::GetPlayerPawn(this, 0);

    // Here's how to cast a ray

    // UWorld* World = GetWorld();
    // FHitResult HitResult;
    // FCollisionQueryParams Params;
    // FVector Start = Grid->GetCellPosition(CellRef);		// need a ray start
    // FVector End = PlayerPawn->GetActorLocation();		// need a ray end
    // Start.Z = End.Z;		// Hack: we don't have Z information in the grid actor -- take the player's z value and raycast against that
    // Add any actors that should be ignored by the raycast by calling
    // Params.AddIgnoredActor(PlayerPawn);			// Probably want to ignore the player pawn
    // Params.AddIgnoredActor(OwnerPawn);			// Probably want to ignore the AI themself
    // bool bHitSomething = World->LineTraceSingleByChannel(HitResult, Start, End, ECollisionChannel::ECC_Visibility, Params);
    // If bHitSomething is false, then we have a clear LOS
}