# Git conventions

## Commits

- Commit names should be short, capitalized, and in imperative case (Eg, `Fix arm PID` or `Create intake subsystem`)
- Commits should contain one edit, not 10 edits
    - One edit does not mean one file, but one feature that was changed/fixed/added

## Branches

- Base branch should contain subsystem name (Eg, `arm` or `swerve`) ***or*** "global feature" name (Eg, `auto`)
- Child branches should:
    - Be appended to parent branch's name with a `.`
    - Describe the feature being worked on (Eg, `arm.pid` or `swerve.vision-odometry`)
- Names should not be long!
    - Names with multiple words should be connected with `-` (Eg, `vision-odometry`)
- Competition branches should be branched off of and PR'ed into main only
- Competition branches are named after their event code (Eg, `BCVI` or `ISDE1`)

## Pull requests

### Title

- Descriptive and concise
- Subsystem: Feature (similar to the branch naming scheme but with colon)
    - Eg: `Arm: Add PID`, `Arm: Fix PID`, `Intake: Fix intake velocity`
- Competition branch PR's should be named in the following way: `[Year] [Eventcode]`
    - Eg: `2025 BCVI` or `2025 ISDE1`

### Description

- Must be descriptive:
    - Include what you have changed / improved in bullet points
Eg:
```
- Adjust arm home position
```
```
- Retune arm PID constants
```
- Competition branch PR's should include a brief bullet-point summary of what was changed
Eg:
```
- Update controller bindings
- Add LED light states for element position
- Add slew limiter for strafe
- Adjust auton paths
- Adjust OTF waypoints
```

### Reviewers

- MINIMUM 2 prog heads
    1. Programming captain
    2. Relevant programming subsystem head
- MINIMUM 1 mentor

### Assignees

- Developers of the branch MUST be assigned to the PR

***REVIEWERS AND ASSIGNEES MUST BE ASSIGNED FOR MAIN***

### Notes

- PR's should not be massive and contain 10 features. It should be specific to one feature only.
- PR's should ***only*** be created when merging subsystems to `main`

## Merges

Always merge (***NOT PR***) feature branch to parent branch before creating a PR to merge subsystem branch into `main`

## General notes

When committing the merge, Git should automatically generate a commit message. Please use those default messeages to maintain consistency.

Eg, `Merge branch 'a' into b` or `Merge pull request #1 from b`
