#pragma once

#include "CoreMinimal.h"
#include "Math/MathFwd.h"
#include "GAGridMap.h"
#include "GAGridActor.generated.h"

class UBoxComponent;
class USceneComponent;
class UProceduralMeshComponent;
class UTexture2D;

UENUM(BlueprintType, meta = (Bitflags, UseEnumValuesAsMaskValuesInEditor = "true"))
enum class ECellData : uint8
{
	CellDataNone = 0,
	CellDataTraversable = 1 << 0
};
ENUM_CLASS_FLAGS(ECellData);


USTRUCT(BlueprintType)
struct FCellRef
{
	GENERATED_BODY()

	FCellRef() : X(INDEX_NONE), Y(INDEX_NONE) {}
	FCellRef(int32 Xin, int32 Yin) : X(Xin), Y(Yin) {}

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 X;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 Y;

	// Note: can't add specifiers, or call from blueprint, because UStructs
	// don't get UFunctions in Unreal
	bool IsValid() const
	{
		return (X >= 0) && (Y >= 0);
	}

	bool operator<(const FCellRef& Other) const
	{
		return (Y < Other.Y) || (Y == Other.Y && X < Other.X);
	}


	// Returns a "cell space" distance between cells
	float Distance(const FCellRef& OtherCell) const
	{
		int32 DX = (X - OtherCell.X);
		int32 DY = (Y - OtherCell.Y);
		return FMath::Sqrt(float(DX * DX + DY * DY));
	}

	// Tests equality between two cells. Needed to use FCellRef in a TMap
	bool operator==(const FCellRef& CellRef) const
	{
		return (X == CellRef.X) && (Y == CellRef.Y);
	}

	// Hash value. Needed to use FCellRef in a TMap
	friend inline uint32 GetTypeHash(const FCellRef& Cell)
	{
		return FCrc::MemCrc32(&Cell, sizeof(FCellRef));
	}

	static FCellRef Invalid;
};


UCLASS(BlueprintType, Blueprintable)
class AGAGridActor : public AActor 
{
	GENERATED_BODY()

public:
	AGAGridActor(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

	// X dimension
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	int32 XCount;

	// Y dimension
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	int32 YCount;

	// Size of a cell
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float CellScale;

	// Calculated
	UPROPERTY()
	FVector2D HalfExtents;

	// Size of a cell
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	bool bDebug;

	// Root component
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	TObjectPtr<USceneComponent> SceneComponent;

	// Data
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
	TArray<ECellData> Data;

	// Data
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
	TArray<float> HeightData;

	virtual void PostLoad() override;

#if WITH_EDITORONLY_DATA
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
	void RefreshBoxComponent();

	// Debug bounds visualization
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	TObjectPtr<UBoxComponent> BoxComponent;
#endif //WITH_EDITORONLY_DATA

private:
	ECellData* GetData() { return Data.GetData(); }
	float* GetHeightData() { return HeightData.GetData(); }
	int32 GetCellCount() { return XCount*YCount; }

	void RefreshDerivedValues();

public:
	bool ResetData();

	// Accessors --------------------------------

	// Return the cell the given point is inside of
	// If bClamp = true, then any point outside of the grid will be clamped to the bounds of the grid
	// Otherwise, if the point is outside the grid, it will return FCellRef::Invalid
	UFUNCTION(BlueprintCallable)
	FCellRef GetCellRef(const FVector& Point, bool bClamp = false) const;

	// Get the world position of the center of the given cell
	UFUNCTION(BlueprintCallable)
	FVector GetCellPosition(const FCellRef& CellRef) const;

	UFUNCTION(BlueprintCallable)
	bool IsCellRefInBounds(const FCellRef& CellRef) const;

	// Get the grid-space position of the center of the given cell
	// Note, grid-space is a bit of a weird idea.
	// In actor space, (0, 0) is the center of the grid
	// In grid space, (0, 0) is the "top left" corner of the grid, i.e. min corner of the (0, 0) cell
	// Getting from actor-space to grid space is easy -- add HalfExtents to the coordinates,
	// where HalfExtents is 0.5 the total width and height of the grid
	FVector2D GetCellGridSpacePosition(const FCellRef& CellRef) const;


	// Return the flattened index of the cell
	// The assumes a X-major ordering of the data array.
	// i.e. if we had a three by three grid, the flattened array would have the data in this order
	//		(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1), (0, 2), (1, 2), (2, 2)
	// Put another way, all the values in a given X-row are stored in consecutive spans of memory
	UFUNCTION(BlueprintCallable)
	FORCEINLINE int32 CellRefToIndex(const FCellRef& CellRef) const { return CellRef.Y * XCount + CellRef.X; }

	// Get the flags associated with the given cell reference
	UFUNCTION(BlueprintCallable)
	ECellData GetCellData(const FCellRef &CellRef) const;

	// Get the flags associated with the given cell reference
	UFUNCTION(BlueprintCallable)
	float GetCellHeightData(const FCellRef &CellRef) const;

	// Returns the bounds of the given box in cell indices
	// Note, assumes the Box is in grid-space already
	// Returns an invalid rectangle if the Box and the grid are disjoint
	bool GridSpaceBoundsToRect2D(const FBox2D &Box, FIntRect& RectOut) const;


	// Data from NavSystem --------------------------------

	UFUNCTION(BlueprintCallable)
	bool RefreshDataFromNav();

	// Debugging and Visualization --------------------------------
	UPROPERTY(EditAnywhere)
	FGAGridMap DebugGridMap;

	// Debug mesh component
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	TObjectPtr<UProceduralMeshComponent> DebugMeshComponent;

	UPROPERTY(EditAnywhere)
	float DebugMeshZOffset;

	UPROPERTY(EditAnywhere)
	TObjectPtr<UMaterialInterface> DebugMaterial;

	UFUNCTION(BlueprintCallable)
	bool RefreshDebugMesh();

	UFUNCTION(BlueprintCallable)
	bool RefreshDebugTexture();

};