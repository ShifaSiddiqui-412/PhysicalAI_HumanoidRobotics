# Data Model: Docusaurus Educational Site for ROS 2 Module

**Feature**: 1-ros2-robotic-system
**Created**: 2025-12-23
**Status**: Draft

## Site Structure Model

### Module Entity
- **module_id**: string (unique identifier for the module)
- **title**: string (display title of the module)
- **description**: string (brief description of the module content)
- **chapters**: Chapter[] (collection of chapters in the module)
- **prerequisites**: string[] (knowledge required before starting)
- **learning_objectives**: string[] (what students will learn)
- **estimated_duration**: number (estimated time to complete in minutes)
- **difficulty_level**: enum ("beginner", "intermediate", "advanced")

### Chapter Entity
- **chapter_id**: string (unique identifier for the chapter)
- **title**: string (display title of the chapter)
- **content_path**: string (path to the markdown file)
- **module_id**: string (reference to parent module)
- **order**: number (position within the module)
- **prerequisites**: string[] (specific knowledge needed for this chapter)
- **learning_objectives**: string[] (specific objectives for this chapter)
- **sections**: Section[] (collection of content sections)
- **code_examples**: CodeExample[] (collection of code examples in chapter)

### Section Entity
- **section_id**: string (unique identifier for the section)
- **title**: string (display title of the section)
- **content_path**: string (path to the markdown file)
- **chapter_id**: string (reference to parent chapter)
- **order**: number (position within the chapter)
- **section_type**: enum ("text", "code", "diagram", "exercise", "summary")
- **estimated_duration**: number (estimated time to read/complete in minutes)

### CodeExample Entity
- **example_id**: string (unique identifier for the example)
- **title**: string (brief description of the example)
- **language**: string (programming language, e.g., "python", "xml")
- **code_content**: string (the actual code content)
- **chapter_id**: string (reference to parent chapter)
- **explanation**: string (description of what the code does)
- **file_path**: string (where this example would be saved if external)

## Content Models

### DocumentationFile Entity
- **file_id**: string (unique identifier for the file)
- **file_path**: string (relative path from docs root)
- **file_type**: enum ("md", "mdx", "png", "jpg", "svg", "other")
- **title**: string (display title for the file)
- **description**: string (brief description of the file content)
- **related_entities**: string[] (IDs of related modules/chapters/sections)
- **last_modified**: date (timestamp of last update)
- **author**: string (author of the content)

### NavigationItem Entity
- **nav_id**: string (unique identifier for the navigation item)
- **label**: string (display text for the navigation item)
- **to**: string (destination path)
- **href**: string (external link if applicable)
- **items**: NavigationItem[] (sub-items in the navigation)
- **order**: number (position in the navigation hierarchy)
- **type**: enum ("link", "category", "doc", "api")

## Search and Metadata Models

### SearchIndex Entity
- **index_id**: string (unique identifier for the search entry)
- **title**: string (title of the indexed content)
- **content**: string (processed text content for search)
- **url**: string (URL path to the content)
- **section**: string (module/chapter/section context)
- **tags**: string[] (associated tags for filtering)
- **last_updated**: date (timestamp of last index update)

### MetaData Entity
- **meta_id**: string (unique identifier for metadata)
- **page_path**: string (path to the page)
- **title**: string (HTML title)
- **description**: string (meta description)
- **keywords**: string[] (meta keywords)
- **og_title**: string (Open Graph title)
- **og_description**: string (Open Graph description)
- **og_image**: string (Open Graph image path)
- **author**: string (content author)

## Validation Rules

### Module Validation
- Title must be 5-100 characters
- Description must be 10-500 characters
- Must have at least one chapter
- Learning objectives must be 3-10 items
- Estimated duration must be positive

### Chapter Validation
- Title must be 5-100 characters
- Must belong to a valid module
- Order must be positive integer
- Learning objectives must be 1-5 items
- Must have content file path

### Content Validation
- File paths must exist in the documentation directory
- Markdown files must be valid
- Code examples must have valid syntax
- All navigation links must be resolvable