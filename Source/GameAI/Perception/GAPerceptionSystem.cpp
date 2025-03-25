#include "GAPerceptionSystem.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/GameModeBase.h"

UGAPerceptionSystem::UGAPerceptionSystem(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{


}


bool UGAPerceptionSystem::RegisterPerceptionComponent(UGAPerceptionComponent* PerceptionComponent)
{
	PerceptionComponents.AddUnique(PerceptionComponent);
	return true;
}

bool UGAPerceptionSystem::UnregisterPerceptionComponent(UGAPerceptionComponent* PerceptionComponent)
{
	return PerceptionComponents.Remove(PerceptionComponent) > 0;
}


bool UGAPerceptionSystem::RegisterTargetComponent(UGATargetComponent* TargetComponent)
{
	TargetComponents.AddUnique(TargetComponent);
	return true;
}

bool UGAPerceptionSystem::UnregisterTargetComponent(UGATargetComponent* TargetComponent)
{
	return TargetComponents.Remove(TargetComponent) > 0;
}


UGAPerceptionSystem* UGAPerceptionSystem::GetPerceptionSystem(const UObject* WorldContextObject)
{
	UGAPerceptionSystem* Result = NULL;
	AGameModeBase *GameMode = UGameplayStatics::GetGameMode(WorldContextObject);
	if (GameMode)
	{
		Result = GameMode->GetComponentByClass<UGAPerceptionSystem>();
	}

	return Result;
}
