# Data Model: Module 4: Vision-Language-Action (VLA)

## Entity: Voice Command
- **name**: string (unique identifier for the command)
- **audio_data**: binary (raw audio input data)
- **transcript**: string (text transcription of the voice command)
- **intent**: string (extracted intent from the command)
- **confidence_score**: float (confidence level of intent recognition)
- **timestamp**: datetime (when the command was received)
- **user_context**: object (information about the user issuing the command)

## Entity: Cognitive Plan
- **plan_id**: string (unique identifier for the plan)
- **original_command**: string (the original natural language command)
- **decomposed_steps**: array (sequence of individual action steps)
- **dependencies**: array (dependencies between steps)
- **estimated_duration**: number (estimated time to execute the plan)
- **confidence_level**: float (confidence in the plan's success)
- **status**: enum ("planning", "executing", "completed", "failed")

## Entity: Action Sequence
- **sequence_id**: string (unique identifier for the action sequence)
- **plan_id**: string (reference to the parent cognitive plan)
- **actions**: array (list of individual robot actions)
- **parameters**: object (parameters for each action)
- **execution_order**: array (order in which actions should be executed)
- **error_handling**: object (how to handle errors during execution)
- **validation_criteria**: object (criteria to validate successful execution)

## Entity: Robot Action
- **action_id**: string (unique identifier for the action)
- **action_type**: enum ("movement", "manipulation", "communication", "sensing")
- **target**: string (what the action targets - e.g., "left_arm", "navigation", "gripper")
- **parameters**: object (specific parameters for the action)
- **duration**: number (expected duration of the action)
- **prerequisites**: array (conditions that must be met before execution)
- **success_criteria**: object (how to determine if action was successful)

## Entity: VLA Pipeline
- **pipeline_id**: string (unique identifier for the pipeline instance)
- **voice_command_id**: string (reference to the original voice command)
- **cognitive_plan_id**: string (reference to the cognitive plan)
- **action_sequence_id**: string (reference to the action sequence)
- **status**: enum ("receiving", "processing", "planning", "executing", "completed", "failed")
- **start_time**: datetime (when the pipeline started)
- **end_time**: datetime (when the pipeline completed)
- **execution_log**: array (log of all steps in the pipeline)

## Relationships
- Voice Command → Cognitive Plan (one-to-many: one command may result in multiple plans)
- Cognitive Plan → Action Sequence (one-to-one: each plan maps to one sequence)
- Action Sequence → Robot Action (one-to-many: each sequence contains multiple actions)
- VLA Pipeline → Voice Command, Cognitive Plan, Action Sequence (one-to-one relationships with each)