# Contributing to AST Robile Safety Features

This document outlines the process we use for contributing.

## Workflow

### 1. Branching

- Always create a new branch from the `main` branch for your work. This helps in keeping the `main` branch stable and makes sure all features are reviewed before they are merged.

### 2. Naming Conventions

- Use descriptive branch names to make it easier to understand the purpose of the branch at a glance.
  - **Feature Branches**: Prefix with `feature/`, e.g., `feature/safety-feature`.
  - **Bug Fixes**: Prefix with `bugfix/`, e.g., `bugfix/failed-rotation-test`.
  - **Enhancements**: Prefix with `enhancement/`, e.g., `enhancement/better-documentation`.

### 3. Pull Requests

- When you're ready for feedback, open a Pull Request (PR).
- Fill out the PR template provided below to give reviewers context about the changes.

### 4. Review Process

- Assign the repository owner or designated reviewer(s) to your PR.
- Address any feedback or requested changes from reviewers.

### 5. Merging Changes

- After the review and approval, you can merge the changes into the `main` branch.
- Ensure that your branch is up-to-date with `main` before merging.

## Pull Request Template

```
Title: [Feature/Bugfix/Enhancement] Your Feature/Bug/Enhancement Name

Description:
- Briefly describe the changes you’ve made.
- Include any relevant issue numbers.
- Mention any particular feedback you're looking for, if any.

Changes:
- List the major changes in bullet points.
- Mention any new dependencies or changes to existing ones.

Testing:
- Describe how you tested your changes.
- Include any specific checks you performed.

```

## Helpful Git Commands

- **Clone a repository**: `git clone [url]`
- **Create a new branch**: `git checkout -b [branch-name]`
- **Switch between branches**: `git checkout [branch-name]`
- **Add changes**: `git add .` or `git add [file-name]`
- **Commit changes**: `git commit -m "[commit message]"`
- **Pull latest changes from remote**: `git pull`
- **Push changes to remote**: `git push origin [branch-name]`
- **Update your branch with latest main**: `git merge main` (ensure you are on your branch)
