#include "GAPathComponent.h"
#include "Engine/World.h"
#include "Math/UnrealMathUtility.h"
#include <cfloat>
#include "GameFramework/NavMovementComponent.h"
#include "Kismet/GameplayStatics.h"


UGAPathComponent::UGAPathComponent(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    State = GAPS_None;
    bDestinationValid = false;
    ArrivalDistance = 100.0f;

    // A bit of Unreal magic to make TickComponent below get called
    PrimaryComponentTick.bCanEverTick = true;
}


const AGAGridActor* UGAPathComponent::GetGridActor() const
{
    if (GridActor.Get())
    {
        return GridActor.Get();
    }
    else
    {
        AGAGridActor* Result = NULL;
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


APawn* UGAPathComponent::GetOwnerPawn()
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




void UGAPathComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    if (bDestinationValid)
    {
        RefreshPath();


        if (State == GAPS_Active)
        {
            FollowPath();
        }
    }

    // Super important! Otherwise, unbelievably, the Tick event in Blueprint won't get called

    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}


EGAPathState UGAPathComponent::RefreshPath()
{
    AActor* Owner = GetOwnerPawn();
    FVector StartPoint = Owner->GetActorLocation();
    check(bDestinationValid);

    float DistanceToDestination = FVector::Dist(StartPoint, Destination);

    if (DistanceToDestination <= ArrivalDistance)
    {
        // Yay! We got there!
        State = GAPS_Finished;
    }
    else
    {
        TArray<FPathStep> UnsmoothedSteps;


        Steps.Empty();


        // Replan the path!
        State = AStar(StartPoint, UnsmoothedSteps);


        // To debug A* without smoothing:
        //Steps = UnsmoothedSteps;


        if (State == EGAPathState::GAPS_Active)
        {
            // Smooth the path!
            State = SmoothPath(StartPoint, UnsmoothedSteps, Steps);
        }
    }

    return State;
}


bool UGAPathComponent::PathDijkstraReconstructPath(const FGAGridMap& DistanceMap, const FCellRef& TargetCell, const FCellRef& StartCell, TArray<FPathStep>& OutPath) const
{
    const AGAGridActor* Grid = GetGridActor();
    if (!Grid)
    {
        UE_LOG(LogTemp, Warning, TEXT("ReconstructPathFromDijkstra: Grid actor not found."));
        return false;
    }

    // Clear any previous path.
    OutPath.Empty();

    // Start from the chosen destination cell (renamed here to TargetCell).
    FCellRef CurrentCell = TargetCell;

    // For clarity, we add each step (from the destination back to the start)
    TArray<FPathStep> ReversePath;
    {
        FVector Pos = Grid->GetCellPosition(CurrentCell);
        FPathStep Step;
        Step.Set(Pos, CurrentCell);
        ReversePath.Add(Step);
    }

    // Define neighbor offsets – using 4-connected neighbors.
    const TArray<FIntPoint> NeighborOffsets = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };

    // Trace backwards until we reach the start cell.
    while (CurrentCell != StartCell)
    {
        float CurrentDistance = 0.0f;
        if (!DistanceMap.GetValue(CurrentCell, CurrentDistance))
        {
            UE_LOG(LogTemp, Warning, TEXT("ReconstructPathFromDijkstra: No distance value for cell (%d, %d)"), CurrentCell.X, CurrentCell.Y);
            return false;
        }

        bool bFoundNextStep = false;
        for (const FIntPoint& Offset : NeighborOffsets)
        {
            FCellRef Neighbor(CurrentCell.X + Offset.X, CurrentCell.Y + Offset.Y);
            if (!Neighbor.IsValid())
                continue;

            if (!EnumHasAllFlags(Grid->GetCellData(Neighbor), ECellData::CellDataTraversable))
                continue;

            float NeighborDistance = 0.0f;
            if (!DistanceMap.GetValue(Neighbor, NeighborDistance))
                continue;

            if (NeighborDistance < CurrentDistance - KINDA_SMALL_NUMBER)
            {
                CurrentCell = Neighbor;
                FVector StepPos = Grid->GetCellPosition(CurrentCell);
                FPathStep NextStep;
                NextStep.Set(StepPos, CurrentCell);
                ReversePath.Add(NextStep);
                bFoundNextStep = true;
                break;
            }
        }

        if (!bFoundNextStep)
        {
            UE_LOG(LogTemp, Warning, TEXT("ReconstructPathFromDijkstra: Could not trace back from cell (%d, %d)"), CurrentCell.X, CurrentCell.Y);
            return false;
        }
    }

    Algo::Reverse(ReversePath);
    OutPath = ReversePath;
    return true;
}

bool UGAPathComponent::ReconstructPath(const FGAGridMap& DistanceMap,
    const FCellRef& TargetCell,   
    const FCellRef& StartCell,
    TArray<FPathStep>& OutPath) const
{
    const AGAGridActor* Grid = GetGridActor();
    if (!Grid)
    {
        UE_LOG(LogTemp, Warning, TEXT("Grid actor not found."));
        return false;
    }

    OutPath.Empty();

    FCellRef CurrentCell = TargetCell;

    TArray<FPathStep> ReversePath;
    {
        FVector Pos = Grid->GetCellPosition(CurrentCell);
        FPathStep Step;
        Step.Set(Pos, CurrentCell);
        ReversePath.Add(Step);
    }

    const TArray<FIntPoint> NeighborOffsets = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };

    while (CurrentCell != StartCell)
    {
        float CurrentDistance = 0.0f;
        if (!DistanceMap.GetValue(CurrentCell, CurrentDistance))
        {
            UE_LOG(LogTemp, Warning, TEXT("ReconstructPathFromDijkstra: No distance value for cell (%d, %d)"), CurrentCell.X, CurrentCell.Y);
            return false;
        }

        bool bFoundNextStep = false;
        for (const FIntPoint& Offset : NeighborOffsets)
        {
            FCellRef Neighbor(CurrentCell.X + Offset.X, CurrentCell.Y + Offset.Y);
            if (!Neighbor.IsValid() || !EnumHasAllFlags(Grid->GetCellData(Neighbor), ECellData::CellDataTraversable))
                continue;

            float NeighborDistance;
            if (!DistanceMap.GetValue(Neighbor, NeighborDistance))
            {
                continue;
            }

            if (NeighborDistance < CurrentDistance)
            {
                CurrentCell = Neighbor;
                FVector StepPos = Grid->GetCellPosition(CurrentCell);
                FPathStep NextStep;
                NextStep.Set(StepPos, CurrentCell);
                ReversePath.Add(NextStep);
                bFoundNextStep = true;
                break;
            }
        }

        if (!bFoundNextStep)
        {
            return false;
        }
    }

    Algo::Reverse(ReversePath);
    OutPath = ReversePath;
    return true;
}



bool UGAPathComponent::Dijkstra(const FVector& StartPoint, FGAGridMap& DistanceMap) const
{
    const AGAGridActor* Grid = GetGridActor();
    if (!Grid)
    {
        UE_LOG(LogTemp, Warning, TEXT("Grid actor not found! "));
        return false;
    }
    FCellRef StartCell = Grid->GetCellRef(StartPoint);
    if (!StartCell.IsValid())
    {
        UE_LOG(LogTemp, Warning, TEXT("Invalid starting cell! "));
        return false;
    }

    DistanceMap.ResetData(FLT_MAX);
    DistanceMap.SetValue(StartCell, 0.0f);

    struct FQueueElement
    {
        FCellRef Cell;
        float Distance;
    };

    TArray<FQueueElement> Queue;
    {
        FQueueElement StartElement;
        StartElement.Cell = StartCell;
        StartElement.Distance = 0.0f;
        Queue.Add(StartElement);
    }
    const TArray<FIntPoint> NeighborOffsets = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };

    while (!Queue.IsEmpty())
    {
        int32 MinIndex = 0;
        float MinDistance = Queue[0].Distance;
        for (int32 i = 1; i < Queue.Num(); i++)
        {
            if (Queue[i].Distance < MinDistance)
            {
                MinDistance = Queue[i].Distance;
                MinIndex = i;
            }
        }
        FQueueElement Current = Queue[MinIndex];
        Queue.RemoveAt(MinIndex);

        for (const FIntPoint& Offset : NeighborOffsets)
        {
            FCellRef Neighbor(Current.Cell.X + Offset.X, Current.Cell.Y + Offset.Y);
            if (!Neighbor.IsValid())
            {
                continue;
            }
            if (!EnumHasAllFlags(Grid->GetCellData(Neighbor), ECellData::CellDataTraversable))
            {
                continue;
            }

            FVector CurrentPos = Grid->GetCellPosition(Current.Cell);
            FVector NeighborPos = Grid->GetCellPosition(Neighbor);
            float EdgeDistance = FVector::Dist(CurrentPos, NeighborPos);
            float NewDistance = Current.Distance + EdgeDistance;

            float OldDistance;
            if (DistanceMap.GetValue(Neighbor, OldDistance))
            {
                if (NewDistance < OldDistance)
                {
                    DistanceMap.SetValue(Neighbor, NewDistance);

                    FQueueElement Elem;
                    Elem.Cell = Neighbor;
                    Elem.Distance = NewDistance;
                    Queue.Add(Elem);
                }
            }
        }
    }
    return true;
}

EGAPathState UGAPathComponent::AStar(const FVector& StartPoint, TArray<FPathStep>& StepsOut) const
{
    const AGAGridActor* Grid = GetGridActor();
    if (!Grid)
    {
        UE_LOG(LogTemp, Warning, TEXT("GridActor is null!"));
        return GAPS_Invalid;
    }

    //get the starting point
    FCellRef StartCell = Grid->GetCellRef(StartPoint);
    if (!StartCell.IsValid() || !DestinationCell.IsValid())
    {
        UE_LOG(LogTemp, Warning, TEXT("Invalid Start or Destination Cell!"));
        return GAPS_Invalid;
    }

    //heap
    TArray<FCellRef> OpenSet;
    //from the start to actor
    TMap<FCellRef, float> GScore;
    //from actor to destination
    TMap<FCellRef, float> HScore;
    TMap<FCellRef, FCellRef> CameFrom;

    //score calculation
    GScore.Add(StartCell, 0.0f);
    HScore.Add(StartCell, FVector::Dist(Grid->GetCellPosition(StartCell), Grid->GetCellPosition(DestinationCell)));

    //filling up the heap based on the scores
    OpenSet.HeapPush(StartCell, [&GScore, &HScore](const FCellRef& A, const FCellRef& B) {
        return (GScore[A] + HScore[A]) < (GScore[B] + HScore[B]);
        });

    while (OpenSet.Num() > 0)
    {
        //pop the cell with the lowest f score
        FCellRef CurrentCell;
        OpenSet.Heapify([&GScore, &HScore](const FCellRef& A, const FCellRef& B) {
            return (GScore[A] + HScore[A]) < (GScore[B] + HScore[B]);
            });


        //pop the top element
        OpenSet.HeapPop(CurrentCell, EAllowShrinking::Yes);

        //destination reached
        if (CurrentCell == DestinationCell)
        {
            FCellRef Step = CurrentCell;
            while (CameFrom.Contains(Step))
            {
                FVector StepPosition = Grid->GetCellPosition(Step);
                FPathStep PathStep;
                PathStep.Set(StepPosition, Step);
                StepsOut.Insert(PathStep, 0);
                Step = CameFrom[Step];
            }
            FPathStep StartPathStep;
            StartPathStep.Set(StartPoint, StartCell);
            StepsOut.Insert(StartPathStep, 0);

            return GAPS_Active;
        }


        //checking neighbors
        static const TArray<FIntPoint> NeighborOffsets = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };
        for (const FIntPoint& Offset : NeighborOffsets)
        {
            FCellRef Neighbor(CurrentCell.X + Offset.X, CurrentCell.Y + Offset.Y);


            //making sure they are valid and traversable
            if (!Neighbor.IsValid() || !EnumHasAllFlags(Grid->GetCellData(Neighbor), ECellData::CellDataTraversable))
            {
                continue;
            }

            float TentativeGScore = GScore[CurrentCell] + FVector::Dist(Grid->GetCellPosition(CurrentCell), Grid->GetCellPosition(Neighbor));

            if (!GScore.Contains(Neighbor) || TentativeGScore < GScore[Neighbor])
            {
                CameFrom.Add(Neighbor, CurrentCell);
                GScore.Add(Neighbor, TentativeGScore);
                HScore.Add(Neighbor, FVector::Dist(Grid->GetCellPosition(Neighbor), Grid->GetCellPosition(DestinationCell)));

                if (!OpenSet.Contains(Neighbor))
                {
                    OpenSet.HeapPush(Neighbor, [&GScore, &HScore](const FCellRef& A, const FCellRef& B) {
                        return (GScore[A] + HScore[A]) < (GScore[B] + HScore[B]);
                        });
                }
            }
        }
    }
    return GAPS_Invalid;
}

EGAPathState UGAPathComponent::SmoothPath(const FVector& StartPoint, const TArray<FPathStep>& UnsmoothedSteps, TArray<FPathStep>& SmoothedStepsOut) const
{
    const AGAGridActor* Grid = GetGridActor();
    if (!Grid)
    {
        UE_LOG(LogTemp, Warning, TEXT("GridActor is null! SmoothPath cannot proceed."));
        return GAPS_Invalid;
    }

    int CurrentIndex = 0;
    SmoothedStepsOut.Empty();

    while (CurrentIndex < UnsmoothedSteps.Num())
    {
        FVector CurrentPoint = UnsmoothedSteps[CurrentIndex].Point;
        FVector NextPoint = CurrentPoint;
        int NextIndex = CurrentIndex + 1;

        while (NextIndex < UnsmoothedSteps.Num())
        {
            FVector TestPoint = UnsmoothedSteps[NextIndex].Point;
            if (!LineTrace(CurrentPoint, TestPoint, Grid))
            {
                break;
            }
            NextPoint = TestPoint;
            NextIndex++;
        }

        FPathStep Step;
        Step.Set(NextPoint, Grid->GetCellRef(NextPoint));
        SmoothedStepsOut.Add(Step);
        CurrentIndex = NextIndex;
    }

    return GAPS_Active; 
}

bool UGAPathComponent::LineTrace(const FVector& Start, const FVector& End, const AGAGridActor* Grid) const
{
    // Convert start and end points to their respective cell references
    FCellRef StartCell = Grid->GetCellRef(Start);
    FCellRef EndCell = Grid->GetCellRef(End);

    // Validate the cells
    if (!StartCell.IsValid() || !EndCell.IsValid())
    {
        return false; // Invalid cells
    }

    int X = StartCell.X, Y = StartCell.Y;
    int DeltaX = FMath::Abs(EndCell.X - X);
    int DeltaY = FMath::Abs(EndCell.Y - Y);
    int StepX = X < EndCell.X ? 1 : -1;
    int StepY = Y < EndCell.Y ? 1 : -1;

    int Error = DeltaX - DeltaY;

    while (X != EndCell.X || Y != EndCell.Y)
    {
        FCellRef CurrentCell(X, Y);
        if (!EnumHasAllFlags(Grid->GetCellData(CurrentCell), ECellData::CellDataTraversable))
        {
            return false; 
        }
        int Error2 = Error * 2;

        if (Error2 > -DeltaY)
        {
            Error -= DeltaY;
            X += StepX;
        }

        if (Error2 < DeltaX)
        {
            Error += DeltaX;
            Y += StepY;
        }
    }

    return true;
}


void UGAPathComponent::FollowPath()
{
    AActor* Owner = GetOwnerPawn();
    FVector StartPoint = Owner->GetActorLocation();

    check(State == GAPS_Active);
    check(Steps.Num() > 0);

    // Popping the first step 
    float DistanceFromStep = FVector::Dist(StartPoint, Steps[0].Point);
    if (DistanceFromStep <= ArrivalDistance)
    {
        Steps.RemoveAt(0);

        if (Steps.Num() == 0)
        {
            State = GAPS_Finished;
            return;
        }
    }

    FVector V = Steps[0].Point - StartPoint;
    V.Normalize();

    UNavMovementComponent* MovementComponent = Owner->FindComponentByClass<UNavMovementComponent>();
    if (MovementComponent)
    {
        MovementComponent->RequestPathMove(V);
    }
}


EGAPathState UGAPathComponent::SetDestination(const FVector& DestinationPoint)
{
    Destination = DestinationPoint;

    State = GAPS_Invalid;
    bDestinationValid = true;

    const AGAGridActor* Grid = GetGridActor();
    if (Grid)
    {
        FCellRef CellRef = Grid->GetCellRef(Destination);
        if (CellRef.IsValid())
        {
            DestinationCell = CellRef;
            bDestinationValid = true;
            RefreshPath();
        }
    }
    return State;
}