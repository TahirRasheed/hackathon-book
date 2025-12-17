# Urdu Translation Workflow

## Overview

This document describes how to generate and maintain Urdu (اردو) translations for Module 1 chapters using Docusaurus i18n framework.

## Directory Structure

```
docs/
├── module-1/
│   ├── 00-intro.md (English)
│   ├── 01-communication.md (English)
│   └── 02-python-agents.md (English)

i18n/
└── ur/
    └── docusaurus-plugin-content-docs/
        └── current/
            └── module-1/
                ├── 00-intro.md (اردو)
                ├── 01-communication.md (اردو)
                └── 02-python-agents.md (اردو)
```

## Translation Process

### Step 1: Create English Chapter

1. Write the chapter in English in `docs/module-1/`
2. Include proper markdown formatting (headers, code blocks, lists)
3. Ensure code examples are properly formatted in markdown code blocks

### Step 2: Generate Urdu Translation

For each English chapter, create a corresponding Urdu translation:

1. Copy the English markdown file
2. Translate content to Urdu:
   - Keep code examples unchanged (code is language-agnostic)
   - Translate all explanatory text
   - Translate headings and labels
   - Translate review questions
3. Place translated file in: `i18n/ur/docusaurus-plugin-content-docs/current/module-1/`

### Step 3: Verify RTL Rendering

1. Build Docusaurus locally: `npm run build`
2. Start development server: `npm start`
3. Switch to Urdu language in the top navigation
4. Verify:
   - Text displays right-to-left (RTL)
   - Code blocks display left-to-right (LTR)
   - All content is readable

## Translation Guidelines

### Language-Specific Considerations

**Preserve (Don't Translate)**:
- ROS 2 technical terms (Node, Topic, Service, Publisher, Subscriber)
- Package names (sensor_msgs, geometry_msgs, rclpy)
- File paths and code examples
- Proper nouns (Boston Dynamics, Tesla, Toyota, Ubuntu)
- URLs and links

**Translate (With Explanation)**:
- Concepts (explain ROS 2 terms in Urdu context)
- Learning objectives and outcomes
- Practical descriptions
- Review questions
- Examples that use English words (provide Urdu explanation)

### Markdown Formatting

When translating Urdu text:
- Maintain exact same heading levels (# ## ### etc.)
- Keep the same code block structure
- Preserve list formatting
- Keep table layouts identical

Example (English):
```markdown
## Learning Objectives

After this chapter, you will be able to:
- Explain what is middleware
- Describe ROS 2's role in humanoid robotics
```

Example (Urdu - same structure):
```markdown
## سیکھنے کے مقاصد

اس باب کے بعد، آپ یہ کر سکیں گے:
- middleware کی وضاحت دیں
- ROS 2 کا کردار humanoid robotics میں بتائیں
```

## Tools & Resources

### Translation Tools

- **Manual**: Use online Urdu typing tools (Urdu.net, Tayari.pk)
- **Machine Assisted**: Google Translate (for initial draft, then manual refinement)
- **Professional**: Hire Urdu translator for technical terms accuracy

### Verification

- Use native Urdu speakers to review translations
- Verify technical accuracy (ROS 2 terms should be consistent)
- Test rendering in Docusaurus with RTL enabled

## Checklist for Each Translation

- [ ] English chapter complete and approved
- [ ] All English text translated to Urdu
- [ ] Code examples preserved (unchanged)
- [ ] Technical terms consistent with terminology guide
- [ ] File placed in correct i18n directory
- [ ] Markdown formatting matches English version
- [ ] RTL rendering verified in browser
- [ ] Language switching works correctly
- [ ] Links/references point to correct Urdu sections

## Current Translation Status

| Chapter | English | Urdu | Status |
|---------|---------|------|--------|
| Chapter 1: Intro | 00-intro.md | 00-intro.md | Pending |
| Chapter 2: Communication | 01-communication.md | 01-communication.md | Pending |
| Chapter 3: Python & URDF | 02-python-agents.md | 02-python-agents.md | Pending |

## For Module Maintainers

When updating an English chapter:

1. Update `docs/module-1/XX-chapter.md`
2. Update corresponding Urdu translation: `i18n/ur/docusaurus-plugin-content-docs/current/module-1/XX-chapter.md`
3. Verify both render correctly with `npm start`
4. Test language switching in browser

---

**Last Updated**: 2025-12-17
**Docusaurus Version**: 3.x
**i18n Framework**: Docusaurus native i18n
