#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseStampedConverter.h"
#include "nav_msgs/Path.h"

#include "NavMsgsPathConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UNavMsgsPathConverter : public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_path(bson_t *b, FString key, ROSMessages::nav_msgs::Path * path, bool LogOnErrors = true)
	{
		bool KeyFound = false;
		KeyFound = UStdMsgsHeaderConverter::_bson_extract_child_header(b, TEXT("msg.header"), &path->header);
		if (!KeyFound) return false;

		path->poses = GetTArrayFromBSON<ROSMessages::geometry_msgs::PoseStamped>(key + ".poses", b, KeyFound, [LogOnErrors](FString subKey, bson_t* subMsg, bool& subKeyFound)
		{
			ROSMessages::geometry_msgs::PoseStamped ret;
			subKeyFound = UGeometryMsgsPoseStampedConverter::_bson_extract_child_pose_stamped(subMsg, subKey, &ret, LogOnErrors);
			return ret;
		}, LogOnErrors);
		if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_path(bson_t *b, const char *key, const ROSMessages::nav_msgs::Path * path)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_path(&child, path);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_path(bson_t *b, const ROSMessages::nav_msgs::Path * path)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &(path->header));
		_bson_append_tarray<ROSMessages::geometry_msgs::PoseStamped>(b, "poses", path->poses, [](bson_t* msg, const char* key, const ROSMessages::geometry_msgs::PoseStamped& pose_stamped)
		{
			UGeometryMsgsPoseStampedConverter::_bson_append_child_pose_stamped(msg, key, &pose_stamped);
		});
	}
};
