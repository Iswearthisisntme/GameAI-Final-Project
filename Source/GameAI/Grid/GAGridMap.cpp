#include "GAGridMap.h"
#include "GAGridActor.h"

UE_DISABLE_OPTIMIZATION

// --------------------- FGridBox ---------------------

bool FGridBox::IsValidCell(const FCellRef& Cell) const
{
	return (Cell.X >= MinX) && (Cell.X <= MaxX) && (Cell.Y >= MinY) && (Cell.Y <= MaxY);
}


// --------------------- FGAGridMap ---------------------

FGAGridMap::FGAGridMap() : XCount(INDEX_NONE), YCount(INDEX_NONE), GridBounds()
{
	// we are empty
}


FGAGridMap::FGAGridMap(int32 XCountIn, int32 YCountIn, float InitialValue)
{
	XCount = XCountIn;
	YCount = YCountIn;
	GridBounds = FGridBox(0, XCount - 1, 0, YCount - 1);

	ResetData(InitialValue);
}

FGAGridMap::FGAGridMap(const AGAGridActor* Grid, float InitialValue)
{
	XCount = Grid->XCount;
	YCount = Grid->YCount;
	GridBounds = FGridBox(0, XCount - 1, 0, YCount - 1);

	ResetData(InitialValue);
}

FGAGridMap::FGAGridMap(const AGAGridActor* Grid, const FGridBox& GridBoxIn, float InitialValue)
{
	XCount = Grid->XCount;
	YCount = Grid->YCount;
	GridBounds = GridBoxIn;

	ResetData(InitialValue);
}

void FGAGridMap::ResetData(float InitialValue)
{
	if (GridBounds.IsValid())
	{
		int32 BoxWidth = GridBounds.GetWidth();
		int32 BoxHeight = GridBounds.GetHeight();

		check(BoxWidth > 0);
		check(BoxHeight > 0);

		int32 CellCount = BoxWidth * BoxHeight;
		Data.SetNum(CellCount);

		for (int32 Index = 0; Index < CellCount; Index++)
		{
			Data[Index] = InitialValue;
		}
	}
	else
	{
		Data.Empty();
	}
}


bool FGAGridMap::CellRefToLocal(const FCellRef& Cell, int32& X, int32& Y) const
{
	if (IsValid() && GridBounds.IsValidCell(Cell))
	{
		X = Cell.X - GridBounds.MinX;
		Y = Cell.Y - GridBounds.MinY;
		return true;
	}
	return false;
}

bool FGAGridMap::LocalToCellRef(int32 X, int32 Y, FCellRef& Cell) const
{
	if (IsValid())
	{
		Cell.X = X + GridBounds.MinX;
		Cell.Y = Y + GridBounds.MinY;
		return (Cell.X <= XCount) && (Cell.Y <= YCount);
	}
	return false;
}


bool FGAGridMap::GetValue(const FCellRef& Cell, float &ValueOut) const
{
	int32 X, Y;
	if (CellRefToLocal(Cell, X, Y))
	{
		int32 Index = GridBounds.GetWidth()* Y + X;
		check(Data.IsValidIndex(Index));
		ValueOut = Data[Index];
		return true;
	}
	return false;
}

bool FGAGridMap::SetValue(const FCellRef& Cell, float Value)
{
	int32 X, Y;
	if (CellRefToLocal(Cell, X, Y))
	{
		int32 Index = GridBounds.GetWidth()* Y + X;
		check(Data.IsValidIndex(Index));
		Data[Index] = Value;
		return true;
	}
	return false;
}

bool FGAGridMap::GetMaxValue(float& MaxValueOut, float IgnoreThreshold) const
{
	if (IsValid())
	{
		MaxValueOut = -UE_MAX_FLT;
		for (int32 Index = 0; Index < Data.Num(); Index++)
		{
			float Value = Data[Index];
			if (Value <= IgnoreThreshold)
			{
				MaxValueOut = FMath::Max(MaxValueOut, Data[Index]);
			}
		}
		return true;
	}
	return false;
}



UE_DISABLE_OPTIMIZATION
