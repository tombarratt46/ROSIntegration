// Author: Tim Fronsee <tfronsee21@gmail.com> 
#pragma once 

#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"

#include "RI/Topic.h"

#include "VisionComponent.generated.h"

UCLASS()
class ROSINTEGRATION_API UVisionComponent : public UCameraComponent
{
  GENERATED_BODY()
  
public:
  UVisionComponent();
  ~UVisionComponent();
  
  void SetFramerate(const float _FrameRate);
  void Pause(const bool _Pause = true);
  bool IsPaused() const;
  
  UPROPERTY(EditAnywhere, Category = "Vision Component")
    FString ParentLink; // Defines the link that binds to the image frame.
  UPROPERTY(EditAnywhere, Category = "Vision Component")
    bool DisableTFPublishing; 
  UPROPERTY(EditAnywhere, Category = "Vision Component")
    uint32 Width;
  UPROPERTY(EditAnywhere, Category = "Vision Component")
    uint32 Height;
  UPROPERTY(EditAnywhere, Category = "Vision Component")
    float Framerate;
  UPROPERTY(EditAnywhere, Category = "Vision Component")
    bool UseEngineFramerate; 
  UPROPERTY(EditAnywhere, Category = "Vision Component")
    int32 ServerPort;
    
      //Topic names
  UPROPERTY(EditAnywhere, Category = "Vision Component")
      FString CameraInfoTopic;
  UPROPERTY(EditAnywhere, Category = "Vision Component")
      FString ImageTopic;




  // The cameras for color, depth and objects;
  UPROPERTY(Transient, EditAnywhere, BlueprintReadWrite, Category = "Vision Component")
    USceneCaptureComponent2D * Color;

  
  UPROPERTY(BlueprintReadWrite, Category = "Vision Component")
    FString ImageFrame = TEXT("/unreal_ros/image_frame");
  UPROPERTY(BlueprintReadWrite, Category = "Vision Component")
    FString ImageOpticalFrame = TEXT("/unreal_ros/image_optical_frame");
    
  UPROPERTY()
    UTopic * CameraInfoPublisher;
  UPROPERTY()
   UTopic * ImagePublisher;


protected:
  
  virtual void InitializeComponent() override;
  virtual void BeginPlay() override;
  virtual void TickComponent(float DeltaTime, 
                             enum ELevelTick TickType,
                             FActorComponentTickFunction *TickFunction) override;
  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
   
  float FrameTime, TimePassed;

private:
    
	// Private data container
	class PrivateData;
	PrivateData *Priv;

	UMaterialInstanceDynamic *MaterialDepthInstance;
  
  TArray<FFloat16Color> ImageColor;
  TArray<uint8> DataColor;
  bool Running, Paused;
  
  void ShowFlagsBasicSetting(FEngineShowFlags &ShowFlags) const;
  void ShowFlagsLit(FEngineShowFlags &ShowFlags) const;
  void ReadImage(UTextureRenderTarget2D *RenderTarget, TArray<FFloat16Color> &ImageData) const;
  void ToColorImage(const TArray<FFloat16Color> &ImageData, uint8 *Bytes) const;
  void ProcessColor();

};