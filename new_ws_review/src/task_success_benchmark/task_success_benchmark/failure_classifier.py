#!/usr/bin/env python3
"""
Classifies failures during task execution
"""

# MoveIt error codes
MOVEIT_ERROR_CODES = {
    1: "SUCCESS",
    -1: "FAILURE",
    -2: "PLANNING_FAILED",
    -3: "INVALID_MOTION_PLAN",
    -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -5: "CONTROL_FAILED",
    -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -7: "TIMED_OUT",
    -8: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_TIMESTAMP",
    -19: "INVALID_LINK_NAME",
    -20: "INVALID_OBJECT_NAME",
    -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE",
    -23: "ROBOT_STATE_STALE",
    -24: "SENSOR_INFO_STALE",
    -31: "NO_IK_SOLUTION",
    99999: "GOAL_REJECTED"
}

# Stage mapping for failure location
STAGE_NAMES = {
    1: "approach_pick",
    2: "pick",
    3: "lift",
    4: "transport_mid",
    5: "approach_place",
    6: "place",
    7: "retreat"
}


class FailureClassifier:
    def __init__(self):
        self.failure_counts = {}
    
    def classify(self, error_code, stage_name):
        """
        Classify failure based on error code and stage
        
        Args:
            error_code: MoveIt error code integer
            stage_name: Name of stage where failure occurred
        
        Returns:
            Failure mode string
        """
        error_str = MOVEIT_ERROR_CODES.get(error_code, f"UNKNOWN_{error_code}")
        failure_mode = f"{error_str}_AT_{stage_name.upper()}"
        
        # Update counts
        self.failure_counts[failure_mode] = self.failure_counts.get(failure_mode, 0) + 1
        
        return failure_mode
    
    def get_summary(self):
        """Return failure distribution"""
        return self.failure_counts.copy()
    
    def reset(self):
        """Reset failure counts"""
        self.failure_counts = {}
    
    @staticmethod
    def get_error_description(error_code):
        """Get human-readable error description"""
        return MOVEIT_ERROR_CODES.get(error_code, f"Unknown error code: {error_code}")
