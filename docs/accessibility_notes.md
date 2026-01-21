# Accessibility Notes for Urdu RTL Content

## RTL Formatting (دائیں سے بائیں ترتیب)
- Markdown files should be viewed in an editor that supports RTL direction.
- In HTML exports, use `<div dir="rtl" lang="ur">` for Urdu sections.
- Ensure that lists and numbered sections align correctly according to language direction.

## Font Requirements
- For optimal web display, use:
  ```css
  font-family: "Noto Sans Urdu", "Jameel Noori Nastaleeq", serif;
  ```
- Standard system fonts on Windows (e.g., Urdu Typesetting) are acceptable fallbacks.

## Screen Readers
- Use proper `lang="ur"` attributes in semantic HTML to ensure screen readers switch to Urdu pronunciation.
- Provide descriptive `alt` text for images in both English and Urdu.

## Code Block Safety
- Ensure code blocks (` ``` `) are wrapped in containers that force LTR (Left-to-Right) direction even within an RTL page context.
- Example:
  ```html
  <pre dir="ltr"><code> ... </code></pre>
  ```

## Universal Design
- Use high-contrast colors for Urdu script, as complex characters can be harder to read in low contrast.
- Ensure line-height is sufficient for Urdu characters (typically 1.5 - 1.8x).
