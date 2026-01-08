# Data Model: Docusaurus UI Upgrade

**Feature**: 005-docusaurus-ui-upgrade
**Date**: 2026-01-08
**Purpose**: Define entities, relationships, and schemas for UI components and theme system

---

## Entity Relationship Diagram

```
ThemeConfig
    ├── ColorPalette (light)
    ├── ColorPalette (dark)
    ├── TypographySystem
    ├── SpacingScale
    └── ResponsiveBreakpoints

Documentation Structure
    ├── Module (4 total)
    │   ├── Chapter (multiple per module)
    │   │   └── DocumentPage
    │   └── Metadata (icon, description)
    ├── Sidebar State
    │   ├── ExpandedModules: Module[]
    │   └── CurrentPage: DocumentPage
    ├── Breadcrumb Path
    │   └── BreadcrumbItem[]
    └── Navigation Context
        ├── CurrentModule
        └── CurrentChapter

UserPreferences
    ├── ThemePreference (light | dark)
    ├── SidebarCollapsed: boolean
    └── PersistenceLocation: localStorage

ComponentInterfaces
    ├── Sidebar Props & State
    ├── Breadcrumbs Props
    ├── ThemeToggle Props
    └── SearchBar Props
```

---

## Core Entities

### 1. ThemeConfig

**Purpose**: Central configuration for all design tokens

**Schema**:

```typescript
interface ThemeConfig {
  id: "theme-config";
  version: "1.0.0"; // Semver for token schema changes

  colors: {
    light: ColorPalette;
    dark: ColorPalette;
  };

  typography: TypographySystem;
  spacing: SpacingScale;
  breakpoints: ResponsiveBreakpoints;

  metadata: {
    lastUpdated: ISO8601Date;
    maintainer: string;
    compatibility: {
      docusaurusVersion: "2.x";
      nodeVersion: "18+";
      browserSupport: ["Chrome 120+", "Firefox 121+", "Safari 17+", "Edge 120+"];
    };
  };
}
```

**Instance Example**:

```json
{
  "id": "theme-config",
  "version": "1.0.0",
  "colors": {
    "light": {
      "primary": "#0066cc",
      "secondary": "#6c757d",
      "accent": "#ff6b35",
      "neutral": {
        "50": "#f8f9fa",
        "100": "#e9ecef",
        "200": "#dee2e6",
        "300": "#ced4da",
        "400": "#adb5bd",
        "500": "#6c757d",
        "600": "#495057",
        "700": "#343a40",
        "800": "#212529",
        "900": "#1a1a1a"
      },
      "success": "#28a745",
      "warning": "#ffc107",
      "error": "#dc3545"
    },
    "dark": {
      "primary": "#4da6ff",
      "secondary": "#adb5bd",
      "accent": "#ffb366",
      "neutral": {
        "50": "#1a1a1a",
        "100": "#212529",
        "200": "#343a40",
        "300": "#495057",
        "400": "#6c757d",
        "500": "#adb5bd",
        "600": "#ced4da",
        "700": "#dee2e6",
        "800": "#e9ecef",
        "900": "#f8f9fa"
      },
      "success": "#52c41a",
      "warning": "#faad14",
      "error": "#ff7875"
    }
  },
  "typography": {
    "fonts": {
      "body": "-apple-system, BlinkMacSystemFont, \"Segoe UI\", Roboto, \"Helvetica Neue\", Arial, sans-serif",
      "heading": "-apple-system, BlinkMacSystemFont, \"Segoe UI\", Roboto, \"Helvetica Neue\", Arial, sans-serif",
      "code": "\"SF Mono\", Monaco, \"Cascadia Code\", \"Roboto Mono\", Consolas, \"Courier New\", monospace"
    },
    "sizes": {
      "xs": "12px",
      "sm": "14px",
      "base": "16px",
      "lg": "18px",
      "xl": "20px",
      "2xl": "24px",
      "3xl": "28px",
      "4xl": "32px"
    },
    "weights": {
      "normal": 400,
      "semibold": 600,
      "bold": 700
    },
    "lineHeights": {
      "tight": 1.3,
      "normal": 1.5,
      "relaxed": 1.6,
      "loose": 1.75
    }
  },
  "spacing": {
    "base": "8px",
    "scale": [0, 4, 8, 12, 16, 24, 32, 48, 64, 96, 128]
  },
  "breakpoints": {
    "mobile": "320px",
    "tablet": "768px",
    "desktop": "1024px",
    "large": "1440px"
  },
  "metadata": {
    "lastUpdated": "2026-01-08",
    "maintainer": "robotics-book-team",
    "compatibility": {
      "docusaurusVersion": "2.x",
      "nodeVersion": "18+",
      "browserSupport": ["Chrome 120+", "Firefox 121+", "Safari 17+", "Edge 120+"]
    }
  }
}
```

---

### 2. ColorPalette

**Purpose**: Define colors for light or dark theme with contrast verification

**Schema**:

```typescript
interface ColorPalette {
  // Primary actions (blue, high contrast)
  primary: HexColor;           // #0066cc (light) or #4da6ff (dark)
  primaryHover: HexColor;      // Darker/lighter on interaction

  // Secondary/supporting (gray)
  secondary: HexColor;         // #6c757d (light) or #adb5bd (dark)

  // Accent color (highlight)
  accent: HexColor;            // #ff6b35 (orange)

  // Neutral scale (grayscale for backgrounds, borders, text)
  neutral: {
    [key: 50 | 100 | 200 | 300 | 400 | 500 | 600 | 700 | 800 | 900]: HexColor;
  };

  // Semantic colors
  success: HexColor;           // #28a745
  warning: HexColor;           // #ffc107
  error: HexColor;             // #dc3545

  // Specific use cases
  textPrimary: HexColor;       // #1a1a1a (light) or #ffffff (dark)
  textSecondary: HexColor;     // #495057 (light) or #adb5bd (dark)
  background: HexColor;        // #ffffff (light) or #1a1a1a (dark)
  surface: HexColor;           // #f8f9fa (light) or #212529 (dark)
  border: HexColor;            // #e0e0e0 (light) or #404040 (dark)
}

type HexColor = string; // "#RRGGBB" format
```

**Validation Rules**:
- All colors must be valid hex format
- Text colors (on backgrounds) must meet 4.5:1 contrast ratio (WCAG AA)
- Colors must be verified with WCAG contrast checker before deployment
- Neutral scale must progress smoothly (50=lightest, 900=darkest in light theme; reversed in dark)

---

### 3. TypographySystem

**Purpose**: Define font families, sizes, weights, and line heights for consistent typography

**Schema**:

```typescript
interface TypographySystem {
  fonts: {
    body: FontFamily;      // System fonts
    heading: FontFamily;   // System fonts (same as body for consistency)
    code: FontFamily;      // Monospace for code blocks
  };

  sizes: {
    xs: CssSize;           // 12px - small UI elements
    sm: CssSize;           // 14px - labels, breadcrumbs
    base: CssSize;         // 16px - body text (default)
    lg: CssSize;           // 18px - large body
    xl: CssSize;           // 20px - subheadings
    2xl: CssSize;          // 24px - H3
    3xl: CssSize;          // 28px - H2
    4xl: CssSize;          // 32px - H1
  };

  weights: {
    normal: 400;           // Body text
    semibold: 600;         // Emphasis, sidebar items
    bold: 700;             // Headings, strong
  };

  lineHeights: {
    tight: 1.3;            // Headings
    normal: 1.5;           // UI text
    relaxed: 1.6;          // Body text
    loose: 1.75;           // Large text
  };
}

type FontFamily = string; // CSS font-family value
type CssSize = string;    // CSS size (e.g., "16px")
```

**Usage Rules**:
- Body text: `font-size: var(--ds-font-size-base); line-height: var(--ds-line-height-relaxed);`
- Headings: `font-size: var(--ds-font-size-3xl); line-height: var(--ds-line-height-tight); font-weight: 700;`
- Code blocks: `font-family: var(--ds-font-family-code); font-size: var(--ds-font-size-sm);`

---

### 4. SpacingScale

**Purpose**: Define consistent spacing (padding, margin) throughout site

**Schema**:

```typescript
interface SpacingScale {
  base: CssSize;              // 8px - base unit
  scale: CssSize[];           // [0, 4, 8, 12, 16, 24, 32, 48, 64, 96, 128] in pixels
}

// Usage: --ds-spacing-xs (4px), --ds-spacing-sm (8px), --ds-spacing-md (16px), etc.
```

**Validation Rules**:
- All spacing values must be multiples of 4px or 8px base unit
- Spacing scale must be used for all padding and margins (no magic numbers)
- Consistency across components enforced via design system review

---

### 5. ResponsiveBreakpoints

**Purpose**: Define viewport breakpoints for responsive design

**Schema**:

```typescript
interface ResponsiveBreakpoints {
  mobile: CssSize;      // 320px - mobile phones
  tablet: CssSize;      // 768px - tablets, large phones
  desktop: CssSize;     // 1024px - desktops
  large: CssSize;       // 1440px - large monitors
}
```

**CSS Media Query Convention**:
```css
/* Mobile first (no media query needed for base styles) */
.component { }

/* Tablet and up */
@media (min-width: var(--ds-breakpoint-tablet)) { }

/* Desktop and up */
@media (min-width: var(--ds-breakpoint-desktop)) { }

/* Large screens */
@media (min-width: var(--ds-breakpoint-large)) { }
```

---

## Documentation Structure Entities

### 6. Module

**Purpose**: Represent one of 4 modules in robotics book

**Schema**:

```typescript
interface Module {
  id: string;                           // "module-1-humanoid-control"
  name: string;                         // "Humanoid Control"
  description: string;                  // Short description
  icon?: string;                        // Icon name or emoji

  chapters: Chapter[];                  // Chapters in this module

  displayOrder: number;                 // Sort order in sidebar

  metadata: {
    createdAt: ISO8601Date;
    lastModified: ISO8601Date;
    author?: string;
  };
}
```

**Concrete Instances**:
```json
[
  {
    "id": "module-1-humanoid-control",
    "name": "Humanoid Control",
    "description": "ROS 2 fundamentals, joint control, kinematics",
    "icon": "🤖",
    "chapters": [...],
    "displayOrder": 1,
    "metadata": {
      "createdAt": "2026-01-08",
      "lastModified": "2026-01-08"
    }
  },
  {
    "id": "module-2-perception-slam",
    "name": "Perception & SLAM",
    "displayOrder": 2
  },
  {
    "id": "module-3-isaac-brain",
    "name": "Isaac Brain",
    "displayOrder": 3
  },
  {
    "id": "module-4-vla",
    "name": "Vision-Language-Action",
    "displayOrder": 4
  }
]
```

---

### 7. Chapter

**Purpose**: Represent a chapter within a module

**Schema**:

```typescript
interface Chapter {
  id: string;                   // "chapter-1-ros2-fundamentals"
  moduleId: string;             // Parent module ID
  title: string;                // Chapter title
  documentId: string;           // Docusaurus document ID (for routing)

  displayOrder: number;         // Sort within module

  isHighlighted?: boolean;      // Current page marker

  metadata: {
    createdAt: ISO8601Date;
    lastModified: ISO8601Date;
    author?: string;
  };
}
```

**Concrete Instances**:
```json
[
  {
    "id": "chapter-1-ros2",
    "moduleId": "module-1-humanoid-control",
    "title": "ROS 2 Fundamentals",
    "documentId": "module-1-humanoid-control/01-ros2-fundamentals",
    "displayOrder": 1,
    "isHighlighted": false
  },
  {
    "id": "chapter-2-joint-control",
    "moduleId": "module-1-humanoid-control",
    "title": "Joint Control",
    "documentId": "module-1-humanoid-control/02-joint-control",
    "displayOrder": 2,
    "isHighlighted": false
  }
]
```

---

### 8. Sidebar State

**Purpose**: Runtime state of navigation sidebar (expanded modules, current page)

**Schema**:

```typescript
interface SidebarState {
  modules: Module[];                    // Full module list
  expandedModules: Set<string>;         // Module IDs expanded
  currentPageId?: string;               // Document ID of current page
  isMobileMenuOpen: boolean;            // Mobile hamburger menu open/closed

  // Methods (stateless, calculated)
  toggleModule(moduleId: string): void;
  navigateTo(documentId: string): void;
  closeMobileMenu(): void;
}
```

**Runtime Example** (React context):
```typescript
const [sidebarState, setSidebarState] = useState<SidebarState>({
  modules: [/* from docusaurus.config.js */],
  expandedModules: new Set(["module-1-humanoid-control"]),
  currentPageId: "module-1-humanoid-control/01-ros2-fundamentals",
  isMobileMenuOpen: false
});
```

---

### 9. Breadcrumb

**Purpose**: Navigation breadcrumb trail for current page

**Schema**:

```typescript
interface Breadcrumb {
  items: BreadcrumbItem[];      // Ordered list from root to current page
}

interface BreadcrumbItem {
  label: string;                // Display text ("Humanoid Control" or "ROS 2 Fundamentals")
  href?: string;                // URL (empty for current page)
  isActive: boolean;            // Current page marker

  ariaLabel?: string;           // For accessibility
}
```

**Concrete Examples**:

Example 1 (on Module 1, Chapter 1 page):
```json
{
  "items": [
    { "label": "Home", "href": "/", "isActive": false },
    { "label": "Humanoid Control", "href": "/docs/module-1-humanoid-control", "isActive": false },
    { "label": "ROS 2 Fundamentals", "href": null, "isActive": true }
  ]
}
```

Example 2 (on Module 4, Chapter 3 page):
```json
{
  "items": [
    { "label": "Home", "href": "/", "isActive": false },
    { "label": "Vision-Language-Action", "href": "/docs/module-4-vla", "isActive": false },
    { "label": "ROS 2 Execution", "href": null, "isActive": true }
  ]
}
```

---

## User Preference Entities

### 10. UserThemePreference

**Purpose**: Persist user's light/dark theme choice

**Schema**:

```typescript
interface UserThemePreference {
  theme: "light" | "dark";              // Selected theme
  persistedAt: ISO8601Timestamp;        // When user set preference
  source: "user-click" | "system-preference";  // How it was set

  storageKey: "app-theme";              // localStorage key
}
```

**localStorage Implementation**:
```javascript
// Read
const saved = localStorage.getItem('app-theme');
const theme = saved || (window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light');

// Write
localStorage.setItem('app-theme', 'dark');

// Listen for system preference changes
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', (e) => {
  if (!localStorage.getItem('app-theme')) {
    applyTheme(e.matches ? 'dark' : 'light');
  }
});
```

---

### 11. UserSidebarPreference

**Purpose**: Persist user's sidebar expansion state (optional)

**Schema**:

```typescript
interface UserSidebarPreference {
  expandedModules: string[];            // Module IDs expanded
  persistedAt: ISO8601Timestamp;
  storageKey: "app-sidebar-state";
}
```

**localStorage Implementation** (optional enhancement):
```javascript
// Save sidebar state
localStorage.setItem('app-sidebar-state', JSON.stringify(['module-1', 'module-2']));

// Load on page load
const saved = JSON.parse(localStorage.getItem('app-sidebar-state') || '[]');
setSidebarState({ expandedModules: new Set(saved) });
```

---

## Component Interface Entities

### 12. SidebarComponent

**Purpose**: Define interface for custom Sidebar React component

**Schema**:

```typescript
interface SidebarComponentProps {
  // Data
  modules: Module[];                    // Module list with chapters
  currentPageId: string;                // Current document ID

  // Callbacks
  onNavigate: (documentId: string) => void;        // Navigate to page
  onToggleModule: (moduleId: string) => void;      // Expand/collapse module
  onMobileMenuClose: () => void;                   // Close mobile menu

  // State
  expandedModules: Set<string>;         // Open modules
  isMobileMenuOpen: boolean;            // Mobile menu visibility
  isMobile: boolean;                    // Viewport detection (width < 768px)
}

interface SidebarComponentState {
  expandedModules: Set<string>;
  isMobileMenuOpen: boolean;
}
```

**Accessibility Props**:
```html
<nav role="navigation" aria-label="Documentation sidebar">
  <div role="menuitem" aria-expanded="true">Module Name</div>
  <a role="menuitem" aria-current="page">Current Page</a>
</nav>
```

---

### 13. BreadcrumbComponent

**Purpose**: Define interface for Breadcrumb React component

**Schema**:

```typescript
interface BreadcrumbComponentProps {
  items: BreadcrumbItem[];              // Breadcrumb items
  separator?: string;                   // Separator between items (default: "/")
  homeHref?: string;                    // Home link (default: "/")
}

interface BreadcrumbComponentState {
  // Stateless functional component
}
```

**Accessibility Props**:
```html
<nav role="navigation" aria-label="Breadcrumb">
  <ol>
    <li><a href="/">Home</a></li>
    <li><span aria-current="page">Current Page</span></li>
  </ol>
</nav>
```

---

### 14. ThemeToggleComponent

**Purpose**: Define interface for theme toggle button/component

**Schema**:

```typescript
interface ThemeToggleComponentProps {
  currentTheme: "light" | "dark";       // Current theme
  onToggle: (newTheme: "light" | "dark") => void;  // Toggle callback

  showLabel?: boolean;                  // Show text label (default: false on mobile)
  ariaLabel?: string;                   // Accessibility label
}

interface ThemeToggleComponentState {
  currentTheme: "light" | "dark";
}
```

**Accessibility Props**:
```html
<button
  aria-label="Toggle dark mode"
  aria-pressed="false"
  onClick={() => onToggle(theme === 'light' ? 'dark' : 'light')}
>
  <Icon />
</button>
```

---

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────┐
│ User Interactions                                   │
├─────────────────────────────────────────────────────┤
│ - Click sidebar module (toggleModule)               │
│ - Click page link (navigateTo)                      │
│ - Toggle theme (toggleTheme)                        │
│ - Click hamburger menu (toggleMobileMenu)           │
└─────────┬───────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────┐
│ React State (Context)                               │
├─────────────────────────────────────────────────────┤
│ - SidebarState                                      │
│ - UserThemePreference                               │
│ - CurrentPageId                                     │
└─────────┬───────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────┐
│ Component Rendering                                 │
├─────────────────────────────────────────────────────┤
│ - Layout (responsive grid)                          │
│ - Sidebar (module/chapter list)                     │
│ - Breadcrumb (navigation trail)                     │
│ - ThemeToggle (light/dark switch)                   │
│ - Main Content (markdown)                           │
└─────────┬───────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────┐
│ CSS Application                                     │
├─────────────────────────────────────────────────────┤
│ - ThemeConfig (CSS variables)                       │
│ - Responsive Breakpoints                           │
│ - Component Styles (CSS modules)                    │
└─────────┬───────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────┐
│ Persistence Layer                                   │
├─────────────────────────────────────────────────────┤
│ - localStorage: app-theme                           │
│ - localStorage: app-sidebar-state (optional)        │
└─────────────────────────────────────────────────────┘
```

---

## Validation & Constraints

| Entity | Validation | Constraint |
|--------|-----------|-----------|
| **ThemeConfig** | Schema version matches | Must validate before deployment |
| **ColorPalette** | 4.5:1 contrast on all text colors | WCAG AA non-negotiable |
| **TypographySystem** | Fonts available system-wide | No web font downloads |
| **SpacingScale** | Multiples of base unit | Enforces visual rhythm |
| **Module** | Unique IDs, valid display order | Prevents sidebar rendering errors |
| **Chapter** | Valid document ID must exist | Prevents broken links |
| **SidebarState** | Set membership validated | Prevents invalid module IDs |
| **BreadcrumbItem** | hrefs must be valid URLs | XSS prevention, routing safety |
| **UserThemePreference** | "light" \| "dark" only | Type safety in persistence |

---

## Status

✅ **Data Model Complete** - Ready for component contract generation and implementation.

All entities are:
- **Documented** with purpose, schema, and examples
- **Validated** with constraint rules
- **Implementable** in React/TypeScript
- **Testable** with clear interfaces
- **Accessible** with ARIA support

Next step: Generate contracts/ with detailed component interface specifications.
