// Copyright Epic Games, Inc. All Rights Reserved.

#include "GameAIGameMode.h"
#include "UObject/ConstructorHelpers.h"

AGameAIGameMode::AGameAIGameMode()
{
	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/Characters/Player/BP_PlayerCharacter"));
	if (PlayerPawnBPClass.Class != NULL)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}
