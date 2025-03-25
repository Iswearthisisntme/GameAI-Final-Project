// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "Logging/LogMacros.h"
#include "GACharacter.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogTemplateAICharacter, Log, All);

// The base class for our AI characters in CS 4150/5150
// Note the use of the UClass specifiers:
//    BlueprintType - you can use this class as the type for a variable in Blueprint
//    Blueprintable - you can create subclasses of this class in Blueprint
// For the full list of UClass specifiers, see https://docs.unrealengine.com/5.3/en-US/class-specifiers/

UCLASS(BlueprintType, Blueprintable, config=Game)
class AGACharacter : public ACharacter
{
	GENERATED_BODY()

public:
	AGACharacter();

	// Initial movement frequency and amplitude
	// Note the use of the UProperty specifiers:
	//    EditAnywhere - In any AGACharacter subclasses, you can override this value from the editor
	//    BlueprintReadWrite - You can read and write the value of the member variable from Blueprint
	//    ClampMin/ ClampMax - Specifies the min and max allowable values for this property (integers and floats only)
	// For the full list of UProperty specifiers, see https://docs.unrealengine.com/5.3/en-US/unreal-engine-uproperties/#propertyspecifiers

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float MoveFrequency;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = -1.0, ClampMax = 1.0f))
	float MoveAmplitude;

protected:
	
	// To add mapping context
	virtual void BeginPlay();

	// Tick every frame
	virtual void Tick(float DeltaSeconds);

};

