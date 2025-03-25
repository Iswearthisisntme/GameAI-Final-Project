#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "GAPerceptionComponent.h"
#include "GATargetComponent.h"
#include "GAPerceptionSystem.generated.h"



UCLASS(BlueprintType, Blueprintable, meta = (BlueprintSpawnableComponent))
class UGAPerceptionSystem : public UActorComponent
{
	GENERATED_UCLASS_BODY()


	UPROPERTY(BlueprintReadOnly)
	TArray<TObjectPtr<UGAPerceptionComponent>> PerceptionComponents;

	UPROPERTY(BlueprintReadOnly)
	TArray<TObjectPtr<UGATargetComponent>> TargetComponents;


	bool RegisterPerceptionComponent(UGAPerceptionComponent* PerceptionComponent);
	bool UnregisterPerceptionComponent(UGAPerceptionComponent* PerceptionComponent);

	bool RegisterTargetComponent(UGATargetComponent* TargetComponent);
	bool UnregisterTargetComponent(UGATargetComponent* TargetComponent);

	TArray<TObjectPtr<UGATargetComponent>>& GetAllTargetComponents() { return TargetComponents; }
	TArray<TObjectPtr<UGAPerceptionComponent>>& GetAllPerceptionComponents() { return PerceptionComponents; }

	static UGAPerceptionSystem* GetPerceptionSystem(const UObject* WorldContextObject);

};