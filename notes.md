# important topics!

/move_base/result
	tells you when you've successfully reached the goal!!!

/tag_detections_pose
	tells you what tags it sees and where it sees it!!! (relative to the camera?)

--------------------------------------------------------------------------------

# algorithm idea:

	1. find out where you are on the map
	2. start navigating around the room by:
		-depending on where you are, set a goal pose to the closest of a list of goal poses
		-when you've reached the goal pose (or realized you can't)
			-spin slowly around 360 degrees, while looking for april tags
			-if we see a NEW /tag_detections_pose during this, stop and try to navigate to it
				-print out the Tag ID
			-finish the 360 degrees
		-pick the next goal and continue
