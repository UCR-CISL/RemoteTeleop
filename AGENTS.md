# Code Preferences
Prefer minimal changes, keep git commits/ diffs concise for easier reviews.
Use subagents as needed to delegate tasks and review their changes.

Prefer object-oriented structure when the code naturally has state, lifecycle, configuration, or multiple interchangeable implementations.

Use classes for:
- components with persistent state
- long-lived services/managers
- hardware/device interfaces
- pipelines with setup/run/cleanup phases
- abstractions that may have multiple backends

Prefer small functions for:
- pure transformations
- simple utility logic
- one-off scripts
- code where a class would only group unrelated helpers

When adding new functionality, first identify whether it belongs in an existing class/module. Avoid creating loose top-level functions when the behavior is part of an existing object's responsibility.

Keep classes focused and testable. Avoid large “god classes”; prefer composition over deep inheritance.

# Repo Description
This codebase implements remote teleoperation code for a car. Legacy Holoscan applications and
fragments have been removed; do not introduce new Holoscan dependencies. There is CARLA code for
simple testing that can be used for reference.

Setup environments and dependencies as needed, but do not automate running anything that involves
both vehicle-side and remote-side. You could possibly test remote-side code, but anything involving
vehicle-side requires in-person work.
