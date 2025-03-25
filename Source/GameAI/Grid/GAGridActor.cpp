#include "GAGridActor.h"

#include "Components/SceneComponent.h"
#include "Components/BoxComponent.h"
#include "ProceduralMeshComponent.h"
#include "NavigationSystem.h"
#include "NavMesh/RecastNavMesh.h"
#include "Engine/Texture2D.h"


UE_DISABLE_OPTIMIZATION



FCellRef FCellRef::Invalid(INDEX_NONE, INDEX_NONE);


AGAGridActor::AGAGridActor(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	XCount = 100;
	YCount = 100;
	CellScale = 100.0f;
	RefreshDerivedValues();

	SceneComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = SceneComponent;

#if WITH_EDITORONLY_DATA
	BoxComponent = CreateDefaultSubobject<UBoxComponent>(TEXT("Box"));
	BoxComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	BoxComponent->bHiddenInGame = true;
	BoxComponent->SetVisibility(true);
	BoxComponent->SetupAttachment(SceneComponent);
	RefreshBoxComponent();
#endif //WITH_EDITORONLY_DATA


	DebugMeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("DebugMesh"));
	DebugMeshComponent->SetupAttachment(RootComponent);
	DebugMeshComponent->SetVisibility(false);

	DebugMeshZOffset = 30.0f;

}

void AGAGridActor::PostLoad()
{
#if WITH_EDITORONLY_DATA
	RefreshBoxComponent();
#endif //WITH_EDITORONLY_DATA

	RefreshDerivedValues();
	Super::PostLoad();
}


#if WITH_EDITORONLY_DATA
void AGAGridActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	FName ChangedPropertyName = PropertyChangedEvent.GetMemberPropertyName();

	// If the width, height or scale change, refresh the scale of the BoxComponent
	if ((ChangedPropertyName == FName("XCount")) || (ChangedPropertyName == FName("YCount")) || (ChangedPropertyName == FName("CellScale")))
	{
		RefreshBoxComponent();
	}

	RefreshDerivedValues();

	Super::PostEditChangeProperty(PropertyChangedEvent);
}

void AGAGridActor::RefreshBoxComponent()
{
	FVector DesiredExtents;

	DesiredExtents.X = 0.5f * float(XCount) * CellScale;
	DesiredExtents.Y = 0.5f * float(YCount) * CellScale;
	DesiredExtents.Z = 10.0f;

	BoxComponent->SetBoxExtent(DesiredExtents);
}

#endif // WITH_EDITORONLY_DATA


void AGAGridActor::RefreshDerivedValues()
{
	// Refresh HalfExtents
	HalfExtents.X = 0.5f * CellScale * float(XCount);
	HalfExtents.Y = 0.5f * CellScale * float(YCount);
}


bool AGAGridActor::ResetData()
{
	bool Result = false;
	int32 CellCount = GetCellCount();
	Data.SetNumZeroed(GetCellCount());
	HeightData.SetNumZeroed(CellCount);

	return Result;
}

// Return the cell the given point is inside of
// If bClamp = true, then any point outside of the grid will be clamped to the bounds of the grid
// Otherwise, if the point is outside the grid, it will return FCellRef::Invalid

FCellRef AGAGridActor::GetCellRef(const FVector& Point, bool bClamp) const
{
	// First, transform the point into grid-local space
	// note, we drop the Z dimension at this point, by casting to a FVector2D
	FTransform GridTransform = GetActorTransform();
	FVector2D LocalPoint = FVector2D(GridTransform.InverseTransformPosition(Point));

	if (bClamp)
	{
		LocalPoint.X = FMath::Clamp(LocalPoint.X, -HalfExtents.X, HalfExtents.X);
		LocalPoint.Y = FMath::Clamp(LocalPoint.Y, -HalfExtents.Y, HalfExtents.Y);
	}
	else if (FMath::Abs(LocalPoint.X) > HalfExtents.X || FMath::Abs(LocalPoint.Y) > HalfExtents.Y)
	{
		return FCellRef::Invalid;
	}

	LocalPoint += HalfExtents;		// Now LocalPoint is relative the to the (0, 0) corner of grid

	// Discretize by dividing by scale and flooring
	// Out of an abundance of caution we also clamp the result to a valid index, to avoid any floating-point issues

	FCellRef Result(
		FMath::Clamp(FMath::FloorToInt32(LocalPoint.X / CellScale), 0, XCount - 1),
		FMath::Clamp(FMath::FloorToInt32(LocalPoint.Y / CellScale), 0, YCount - 1)
	);
	return Result;
}

FVector AGAGridActor::GetCellPosition(const FCellRef& CellRef) const
{
	float HalfScale = 0.5f * CellScale;
	int32 Index = CellRefToIndex(CellRef);

	// Grab the center of the cell, then offset by -HalfExtents, so that it is relative to the center of the grid
	FVector LocalResult;
	LocalResult.X = CellRef.X * CellScale + HalfScale - HalfExtents.X;
	LocalResult.Y = CellRef.Y * CellScale + HalfScale - HalfExtents.Y;
	LocalResult.Z = HeightData.IsValidIndex(Index) ? HeightData[Index] :  0.0f;

	FTransform ActorTransform = GetActorTransform();
	FVector Result = ActorTransform.TransformPosition(LocalResult);
	return Result;
}

bool AGAGridActor::IsCellRefInBounds(const FCellRef& CellRef) const
{
	return (CellRef.X >= 0) && (CellRef.X < XCount) && (CellRef.Y >= 0) && (CellRef.Y < YCount);
}



FVector2D AGAGridActor::GetCellGridSpacePosition(const FCellRef& CellRef) const
{
	float HalfScale = 0.5f * CellScale;

	// Grab the center of the cell, then offset by HalfScale, so that it is relative to the center of the grid
	FVector2D LocalResult;
	LocalResult.X = CellRef.X * CellScale + HalfScale;
	LocalResult.Y = CellRef.Y * CellScale + HalfScale;

	return LocalResult;
}



ECellData AGAGridActor::GetCellData(const FCellRef &CellRef) const
{
	int32 CellIndex = CellRefToIndex(CellRef);
	return Data[CellIndex];
}


float AGAGridActor::GetCellHeightData(const FCellRef& CellRef) const
{
	int32 CellIndex = CellRefToIndex(CellRef);
	return HeightData[CellIndex];
}


bool AGAGridActor::GridSpaceBoundsToRect2D(const FBox2D& Box, FIntRect &RectOut) const
{
	float HalfScale = 0.5f * CellScale;

	RectOut.Min.X = FMath::Max(int32((Box.Min.X + HalfScale) / CellScale), 0);
	RectOut.Max.X = FMath::Min(int32((Box.Max.X - HalfScale) / CellScale), XCount - 1);	
	RectOut.Min.Y = FMath::Max(int32((Box.Min.Y + HalfScale) / CellScale), 0);
	RectOut.Max.Y = FMath::Min(int32((Box.Max.Y - HalfScale) / CellScale), YCount - 1);

	return (RectOut.Min.X <= RectOut.Max.X) && (RectOut.Min.Y <= RectOut.Max.Y);
}


// Data from NavSystem --------------------------------

bool AGAGridActor::RefreshDataFromNav()
{
	bool Result = false;
	UNavigationSystemV1 *NavSystem = UNavigationSystemV1::GetNavigationSystem(this);
	if (NavSystem)
	{
		INavigationDataInterface* NavData = NavSystem->GetMainNavData();		// Note: only using the default nav data here
		const ARecastNavMesh* NavMesh = Cast<ARecastNavMesh>(NavData);
		FTransform ActorTransform = GetActorTransform();

		// Allocate the array and set to 0
		ResetData();

		ECellData* CellData = GetData();
		FVector HalfExtents3D(HalfExtents.X, HalfExtents.Y, 0.0f);

		// Code for extracting nav polys taken from here:
		// https://nerivec.github.io/old-ue4-wiki/pages/ai-navigation-in-c-customize-path-following-every-tick.html

		TArray<FNavTileRef> NavTiles;
		NavMesh->GetAllNavMeshTiles(NavTiles);
		for (FNavTileRef &TileRef : NavTiles)
		{
			const FBox TileBounds = NavMesh->GetNavMeshTileBounds(TileRef);
			if (TileBounds.IsValid)			// reportedly will crash if this is not checked
			{
				TArray<FNavPoly> Polys;

				if (NavMesh->GetPolysInTile(TileRef, Polys))
				{
					for (FNavPoly& NavPoly : Polys)
					{
						TArray<FVector> PolyVerts;
						TArray<FVector> PolyVertsLocal;
						NavNodeRef Ref = NavPoly.Ref;

						// We're converting polys to triangles
						PolyVertsLocal.SetNum(3);

						NavMesh->GetPolyVerts(Ref, PolyVerts);

						// Warning: contrary to what a healthy, well-adjusted individual might expect, nav polys are not planar.

						// We can't do any of the above if we don't have at least 2 verts in the poly
						// (it wouldn't even be a poly at that point)
						if (PolyVerts.Num() <= 2)
						{
							continue;
						}

						for (int32 TriangleIndex= 0; TriangleIndex <= PolyVerts.Num() - 3; TriangleIndex++)
						{
							PolyVertsLocal[0] = PolyVerts[0];
							PolyVertsLocal[1] = PolyVerts[1 + TriangleIndex];
							PolyVertsLocal[2] = PolyVerts[2 + TriangleIndex];

							// transform verts to local space
							for (int32 VertexIndex = 0; VertexIndex < PolyVertsLocal.Num(); VertexIndex++)
							{
								PolyVertsLocal[VertexIndex] = ActorTransform.InverseTransformPosition(PolyVertsLocal[VertexIndex]) + HalfExtents3D;
							}

							{
								FBox2D PolyBounds(EForceInit::ForceInit);
								for (const FVector& Vert : PolyVertsLocal)
								{
									PolyBounds += FVector2D(Vert);
								}

								FIntRect GridBox;
								if (GridSpaceBoundsToRect2D(PolyBounds, GridBox))
								{
									FVector PlaneNormal;
									FVector2D PlaneNormal2D;
									float PlaneD;

									TArray<FVector2D> OutsideVectors;
									OutsideVectors.SetNum(PolyVertsLocal.Num());

									// calculate the plane of the poly (this is all in local grid space)
									// Remember a plane is defined by the equation N.x - d = 0
									// where N is the normal and d is a constant (distance from the origin)

									{
										FVector V0, V1;

										V0 = PolyVertsLocal[1] - PolyVertsLocal[0];
										V1 = PolyVertsLocal[2] - PolyVertsLocal[0];

										PlaneNormal = V1 ^ V0;				// cross product
										PlaneNormal.Normalize();

										PlaneD = PlaneNormal | PolyVertsLocal[0];			// dot product. Note, we know the poly verts are on the plane in question
										PlaneNormal2D = FVector2D(PlaneNormal);
									}

									// This should never happen -- it would suggest a poly that is vertical wall, instead of 
									// a mostly-level floor. Still, we're going to divide by this below, so to be safe...
									if (PlaneNormal.Z == 0.0f)
									{
										continue;
									}

									// Cache off a set of "outside vectors", so that we can test each cell in the box against the
									// bounds of the polygons

									for (int32 V0Index = 0; V0Index < PolyVertsLocal.Num(); V0Index++)
									{
										int32 V1Index = (V0Index + 1) % PolyVertsLocal.Num();
										FVector2D V0V1 = FVector2D(PolyVertsLocal[V1Index] - PolyVertsLocal[V0Index]);

										// Rotate 90 degrees
										FVector2D& OutsideVector = OutsideVectors[V0Index];
										OutsideVector.X = -V0V1.Y;
										OutsideVector.Y = V0V1.X;
									}

									// Grid box now represents the intersection between Grid and the poly in question.
									// Check each cell in the box to see if it's inside the poly

									// Note, this is way inefficient
									// A smarter way would be to use some kind of rasterization approach

									for (int Y = GridBox.Min.Y; Y <= GridBox.Max.Y; Y++)
									{
										for (int X = GridBox.Min.X; X <= GridBox.Max.X; X++)
										{
											FCellRef CellRef(X, Y);
											FVector2D CellCenter = GetCellGridSpacePosition(CellRef);
											bool IsOutside = false;

											for (int32 VIndex = 0; (VIndex < PolyVertsLocal.Num()) && !IsOutside; VIndex++)
											{
												FVector2D V = FVector2D(PolyVertsLocal[VIndex]);
												const FVector2D& OutsideVector = OutsideVectors[VIndex];
												FVector2D V2Cell = CellCenter - V;

												if ((V2Cell | OutsideVector) > 0.0f)		// Dot product
												{
													// the cell is outside this edge
													IsOutside = true;
												}
											}

											if (!IsOutside)
											{
												// turn on the traversable bit
												int32 CellIndex = CellRefToIndex(CellRef);
												bool bFirst = false;

												if (!EnumHasAnyFlags(CellData[CellIndex], ECellData::CellDataTraversable))
												{
													EnumAddFlags(CellData[CellIndex], ECellData::CellDataTraversable);
													bFirst = true;
												}

												// Compute the height based on the local poly verts
												// Find the vertical projection of the CellCenter to the plane
												float H = (PlaneD - (CellCenter | PlaneNormal2D)) / PlaneNormal.Z;

												// See if it's higher than what's already there
												if (bFirst || (H > HeightData[CellIndex]))
												{
													HeightData[CellIndex] = H;
												}

											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}

	return Result;
}


// Debugging and Visualization --------------------------------


bool AGAGridActor::RefreshDebugMesh()
{
	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UV0;
	TArray<FColor> VertexColors;			// can safely leave empty
	TArray<FProcMeshTangent> Tangents;		// can safely leave empty

	int32 VertexCount = (XCount + 1) * (YCount + 1);

	// Generate the vertices array
	// TODO: do a raycast to to figure out where the ground actually is
	{
		Vertices.SetNumUninitialized(VertexCount);

		FVector2D ZeroZeroCorner(
			-float(XCount) * CellScale * 0.5f, 
			-float(YCount) * CellScale * 0.5f);

		int32 Index = 0;

		for (int32 Y = 0; Y <= YCount; Y++)				// note, the use of the <= means we will have ONE MORE row and column or verts than cells
		{
			for (int32 X = 0; X <= XCount; X++)
			{
				FVector VertexPoint;
				float H = 0.0f;
				int32 HCount = 0;

				// computing the height is a bit of an annoyance. Because these verts are at the corners between cells (not at cell centers)
				// we don't have perfect height data for them. But we can approximate by averaging adjacent cell heights. This is okay because
				// it's just for debugging. We will omit non traversable cells though

				FCellRef BottomRight(X, Y);
				FCellRef TopRight(X, Y - 1);
				FCellRef TopLeft(X - 1, Y - 1);
				FCellRef BottomLeft(X - 1, Y);

				if (IsCellRefInBounds(BottomRight) && EnumHasAllFlags(GetCellData(BottomRight), ECellData::CellDataTraversable))
				{
					int32 CellIndex = CellRefToIndex(BottomRight);
					H += HeightData[CellIndex];
					HCount++;
				}

				if (IsCellRefInBounds(TopRight) && EnumHasAllFlags(GetCellData(TopRight), ECellData::CellDataTraversable))
				{
					int32 CellIndex = CellRefToIndex(TopRight);
					H += HeightData[CellIndex];
					HCount++;
				}

				if (IsCellRefInBounds(TopLeft) && EnumHasAllFlags(GetCellData(TopLeft), ECellData::CellDataTraversable))
				{
					int32 CellIndex = CellRefToIndex(TopLeft);
					H += HeightData[CellIndex];
					HCount++;
				}

				if (IsCellRefInBounds(BottomLeft) && EnumHasAllFlags(GetCellData(BottomLeft), ECellData::CellDataTraversable))
				{
					int32 CellIndex = CellRefToIndex(BottomLeft);
					H += HeightData[CellIndex];
					HCount++;
				}

				if (HCount > 0)
				{
					H /= float(HCount);
				}

				VertexPoint.X = float(X) * CellScale + ZeroZeroCorner.X;
				VertexPoint.Y = float(Y) * CellScale + ZeroZeroCorner.Y;
				VertexPoint.Z = H + DebugMeshZOffset;
				Vertices[Index] = VertexPoint;
				Index++;
			}
		}
	}

	// Generate the triangles array with counter-clockwise winding
	{
		int32 TriangleCount = GetCellCount() * 6;		// two triangles per cell
		Triangles.SetNumUninitialized(TriangleCount);

		int32 VertexXCount = XCount + 1;
		int32 TriangleIndex = 0;

		for (int32 Y = 0; Y < YCount; Y++)
		{
			for (int32 X = 0; X < XCount; X++)
			{
				// Don't bother for non traversable cells
				if (EnumHasAllFlags(GetCellData(FCellRef(X, Y)), ECellData::CellDataTraversable))
				{
					// First figure out the four vertices of this cell (in counter-clockwise order)
					// Note: personally, this breaks my brain a bit, but the labels of "bottom" and "left" etc. below are using
					// UE's weird left-hand coordinate system, whereby X is the "right" direction and Y is the "down" direction

					int32 Index0 = Y * VertexXCount + X;		// Top left
					int32 Index1 = Index0 + VertexXCount;		// Bottom left
					int32 Index2 = Index1 + 1;					// Bottom right
					int32 Index3 = Index0 + 1;					// Top right

					// First triangle - bottom right half of the cell
					Triangles[TriangleIndex] = Index0;
					Triangles[TriangleIndex + 1] = Index1;
					Triangles[TriangleIndex + 2] = Index2;

					// Second triangle - top left half of the cell
					Triangles[TriangleIndex + 3] = Index0;
					Triangles[TriangleIndex + 4] = Index2;
					Triangles[TriangleIndex + 5] = Index3;

					TriangleIndex += 6;
				}
			}
		}

		// Original length was an upper limit. Cut it down to the number of triangles we actually set.
		Triangles.SetNum(TriangleIndex, EAllowShrinking::Yes);
	}

	// Generate the UV coordinates for each vertex
	{
		UV0.SetNumUninitialized(VertexCount);
		float DeltaX = 1.0f / float(XCount);
		float DeltaY = 1.0f / float(YCount);

		int32 Index = 0; 

		for (int32 Y = 0; Y <= YCount; Y++)
		{
			for (int32 X = 0; X <= XCount; X++)
			{
				FVector2D UV(
					float(X) * DeltaX,
					float(Y) * DeltaY
				);

				UV0[Index] = UV;
				Index++;
			}
		}
	}

	// Generate the UV coordinates for each vertex
	// For now, assume all are straight up
	{
		Normals.SetNumUninitialized(VertexCount);
		int32 Index = 0;

		for (int32 Y = 0; Y <= YCount; Y++)
		{
			for (int32 X = 0; X <= XCount; X++)
			{
				Normals[Index] = FVector::UpVector;
				Index++;
			}
		}
	}

	DebugMeshComponent->CreateMeshSection(
		0,				// section index
		Vertices,		
		Triangles,
		Normals,
		UV0,
		VertexColors,
		Tangents,
		false  // create collision
	);

	return true;
}

bool AGAGridActor::RefreshDebugTexture()
{
	bool Result = false;

	// Note: this is for debugging the map rendering
	/*
	{
		FGridBox Box(XCount / 4, XCount - XCount / 4, YCount / 4, YCount - YCount / 4);
		DebugGridMap = FGAGridMap(this, Box, 0.0f);
		for (int32 Y = Box.MinY; Y <= Box.MaxY; Y++)
		{
			for (int32 X = Box.MinX; X <= Box.MaxX; X++)
			{
				DebugGridMap.SetValue(FCellRef(X, Y), float((X-Box.MinX) + (Y - Box.MinY)));
			}
		}
	}
	*/

	if (DebugMeshComponent)
	{
		// create a new texture
		UTexture2D *RuntimeTexture = UTexture2D::CreateTransient(XCount, YCount);
		FTexture2DMipMap *FirstMip = &RuntimeTexture->GetPlatformData()->Mips[0];
		FByteBulkData *ImageData = &FirstMip->BulkData;
		uint8* RawImageData = (uint8*)ImageData->Lock(LOCK_READ_WRITE);
		int32 ElementCount = ImageData->GetElementCount();
		int32 ElementSize = ImageData->GetElementSize();

		check(ElementCount == 4 * XCount * YCount);
		check(ElementSize == sizeof(uint8));

		int32 Index = 0;

		if (DebugGridMap.IsValid())
		{
			float MaxValue;
			DebugGridMap.GetMaxValue(MaxValue, BIG_NUMBER);

			for (int32 Y = 0; Y < YCount; Y++)
			{
				for (int32 X = 0; X < XCount; X++)
				{
					FCellRef CellRef(X, Y);
					ECellData CellData = GetCellData(CellRef);
					bool Traversable = EnumHasAllFlags(CellData, ECellData::CellDataTraversable);

					float MapValue;
					bool IsOnMap = DebugGridMap.GetValue(CellRef, MapValue);
					int32 IntVal = 0;
					if (IsOnMap)
					{
						IntVal = FMath::RoundToInt(255.0f * (MapValue / MaxValue));
					}

					// Note: fade from blue to red as we approach the max value in the debug map

					RawImageData[Index] = IsOnMap ? 255 - IntVal : 0;		// blue		Are we on the map or not?
					RawImageData[Index + 1] = Traversable ? 50 : 0;			// green	Are we traversable or not?
					RawImageData[Index + 2] = IntVal;						// red		The value
					RawImageData[Index + 3] = 255;							// alpha

					Index += 4;
				}
			}

		}
		else
		{
			for (int32 Y = 0; Y < YCount; Y++)
			{
				for (int32 X = 0; X < XCount; X++)
				{
					ECellData CellData = GetCellData(FCellRef(X, Y));
					bool Traversable = EnumHasAllFlags(CellData, ECellData::CellDataTraversable);
					uint8 Val = Traversable ? 255 : 0;

					RawImageData[Index] = Val;			// blue 
					RawImageData[Index + 1] = Val;		// green
					RawImageData[Index + 2] = Val;		// red
					RawImageData[Index + 3] = 255;		// alpha

					Index += 4;
				}
			}
		}

		ImageData->Unlock();
		RuntimeTexture->UpdateResource();

		UMaterialInstanceDynamic* DynamicMaterial = DebugMeshComponent->CreateDynamicMaterialInstance(0, DebugMaterial);
		if (DynamicMaterial)
		{
			DynamicMaterial->SetTextureParameterValue("DebugTexture", RuntimeTexture);
			DebugMeshComponent->SetMaterial(0, DynamicMaterial);
		}

		Result = true;
	}

	return Result;
}

UE_ENABLE_OPTIMIZATION
