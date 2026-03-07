# University Presentations — Academic Documents Generator

This folder contains a self-contained Python script that generates six academic documents (PPTX presentations and DOCX reports) for two Telecommunications Engineering undergraduate modules.

## Modules Covered

| Module Code | Module Name | Lecturer |
|---|---|---|
| HTENG 437 | Next Generation Networks | Mr Brian Sadock |
| HTENG 440 | Cybersecurity for Engineers | Mr Brian Sadock |

## Generated Files

Running `generate_all_documents.py` produces the following files in this directory:

| File | Type | Description |
|---|---|---|
| `HTENG437_Topic3.6_Presentation.pptx` | PowerPoint | Topic 3.6 — Advanced Network Concepts (Digital Twins, Internet of Senses, Holographic Comms, 3D Network Architecture) |
| `HTENG437_Topic3.6_Detailed_Report.docx` | Word Document | Detailed report covering all subtopics of HTENG 437 Topic 3.6 |
| `HTENG440_Chapter2_Presentation.pptx` | PowerPoint | Chapter 2 — Threats, Vulnerabilities & Mitigations |
| `HTENG440_Chapter2_Detailed_Report.docx` | Word Document | Detailed report covering all subtopics of HTENG 440 Chapter 2 |
| `HTENG437_Full_Module_Summary.pptx` | PowerPoint | Full module summary for HTENG 437 (all 5 topics, min. 15 slides) |
| `HTENG440_Full_Module_Summary.pptx` | PowerPoint | Full module summary for HTENG 440 (all 5 chapters, min. 15 slides) |

## How to Run

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Generate all documents

```bash
python generate_all_documents.py
```

All six output files will be created in the `university_presentations/` directory.

## Themes

- **HTENG 437** presentations use a **dark navy blue** theme (`#0A1628` background) with white and cyan (`#00D4FF`) accent text.
- **HTENG 440** presentations use a **dark red/black** theme (`#1A0A0A` background) with white and orange (`#FF6B35`) accent text.
- All presentations use **Calibri** font.

## Requirements

- Python 3.7+
- `python-pptx` ≥ 0.6.21
- `python-docx` ≥ 0.8.11
- `Pillow` ≥ 9.0.0
