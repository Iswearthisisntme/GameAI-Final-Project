// Copyright Epic Games, Inc. All Rights Reserved.

#include "GACharacter.h"
#include "Components/CapsuleComponent.h"
#include "GameFramework/CharacterMovementComponent.h"

DEFINE_LOG_CATEGORY(LogTemplateAICharacter);

//////////////////////////////////////////////////////////////////////////
// AGACharacter

AGACharacter::AGACharacter()
{
	// Set size for collision capsule
	GetCapsuleComponent()->InitCapsuleSize(42.f, 96.0f);
		
	// Configure character rotation
	// Should the character rotate towards the direction of movement?
	GetCharacterMovement()->bOrientRotationToMovement = true; // Character moves in the direction of input...	
	// ... or should it take rotation from its controller?
	bUseControllerRotationPitch = false;
	bUseControllerRotationYaw = false;
	bUseControllerRotationRoll = false;

	GetCharacterMovement()->RotationRate = FRotator(0.0f, 500.0f, 0.0f); // ...at this rotation rate

	// Note: For faster iteration times these variables, and many more, can be tweaked in the Character Blueprint
	// instead of recompiling to adjust them
	GetCharacterMovement()->JumpZVelocity = 700.f;
	GetCharacterMovement()->AirControl = 0.35f;
	GetCharacterMovement()->MaxWalkSpeed = 200.f;
	GetCharacterMovement()->MinAnalogWalkSpeed = 20.f;
	GetCharacterMovement()->BrakingDecelerationWalking = 2000.f;
	GetCharacterMovement()->BrakingDecelerationFalling = 1500.0f;

	// Initial movement frequency and amplitude
	MoveFrequency = 1.5f;
	MoveAmplitude = 1.0f;

}

void AGACharacter::BeginPlay()
{
	// Call the base class  
	Super::BeginPlay();

}

void AGACharacter::Tick(float DeltaSeconds)
{
	// Do nothing

	Super::Tick(DeltaSeconds);
}